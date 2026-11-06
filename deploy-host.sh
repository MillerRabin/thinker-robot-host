#!/bin/bash

# Deploy Host Script
# Builds the ESP32 host firmware, uploads it over the air (POST /update), waits for the host to come
# back and checks that /version reports this build. The host cuts the engines when the upload starts
# and reboots after it, so hold the arm.
# A host still running firmware from before the OTA watchdog fix needs a slow upload:
#   RATE=14k ./deploy-host.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST="192.168.1.120"
PIO="${PIO:-$HOME/.platformio/penv/bin/pio}"
BIN_FILE="$SCRIPT_DIR/.pio/build/esp32doit-devkit-v1/firmware.bin"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "🖥️  Host Deployment Script"
echo "==============================================="

cd "$SCRIPT_DIR" || exit 1

echo -e "${BLUE}🔨 Building project...${NC}"
if ! "$PIO" run; then
    echo -e "${RED}❌ Build failed!${NC}"
    exit 1
fi
EXPECTED_BUILD="$(git describe --always --dirty)"
echo -e "${GREEN}✅ Build $EXPECTED_BUILD, $(ls -lh "$BIN_FILE" | awk '{print $5}')${NC}"
echo

echo -e "${BLUE}🔄 Turning off engines...${NC}"
curl -X POST "http://$HOST/set" \
     -H "Content-Type: application/json" \
     -d '{"enginesEnabled": false }' \
     -m 10 --silent --fail
echo

echo -e "${BLUE}📤 Uploading firmware${RATE:+ at $RATE/s}...${NC}"
RESPONSE="$(curl -F "update=@$BIN_FILE" "http://$HOST/update" -m 300 --silent ${RATE:+--limit-rate "$RATE"})"
if [ "$RESPONSE" = "OK" ]; then
    echo -e "${GREEN}✅ Upload accepted${NC}"
elif [ -z "$RESPONSE" ]; then
    # It can reboot into the new image before its "OK" gets out (2026-11-06) - the build check below
    # tells; a host that reset mid-upload comes back with the old build and fails it.
    echo -e "${YELLOW}⚠️  No answer to the upload - checking which build comes back${NC}"
else
    echo -e "${RED}❌ Upload failed (host answered: $RESPONSE)${NC}"
    echo "   An older host firmware resets on a full-speed upload - retry with RATE=14k."
    exit 1
fi

echo -e "${BLUE}⏳ Waiting for the host to come back...${NC}"
for i in $(seq 1 60); do
    sleep 2
    BUILD="$(curl -X POST "http://$HOST/version" -m 2 --silent | sed -n 's/.*"build":"\([^"]*\)".*/\1/p')"
    if [ "$BUILD" = "$EXPECTED_BUILD" ]; then
        echo -e "${GREEN}✅ Host runs build $BUILD${NC}"
        exit 0
    fi
done

echo -e "${RED}❌ Host doesn't report build $EXPECTED_BUILD (last seen: ${BUILD:-no answer})${NC}"
exit 1
