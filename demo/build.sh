#!/usr/bin/env sh
# Builds the demo into demo/www. Then: cd demo/www && python -m http.server, and open http://localhost:8000.
set -e
cd "$(dirname "$0")"
cargo build --release --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/release/dittomo_demo.wasm www/
if command -v wasm-opt >/dev/null 2>&1; then
  wasm-opt -O3 www/dittomo_demo.wasm -o www/dittomo_demo.wasm
fi
ls -l www/dittomo_demo.wasm
