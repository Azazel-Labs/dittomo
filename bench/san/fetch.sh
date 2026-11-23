#!/bin/sh
# Downloads the Mixamo test set of Skeleton-Aware Networks (Aberman et al. 2020; github.com/DeepMotionEditing/
# deep-motion-editing, BSD-2-Clause code), which `cargo bench --bench san` scores dittomo on, into bench/san/data (or
# the directory given), checked against its SHA-256. It's fetched, not committed: the clips are Adobe Mixamo content
# as SAN's authors exported them, 106 clips on each of eight characters, as BVH.
set -eu

dir=${1:-"$(dirname "$0")/data"}
mkdir -p "$dir"
archive="$dir/test_set.tar.bz2"
want=de06d429e8c7f04dc62be8bd0415ce515afd5fdb886536bbaf1c8f3a9eedc2c1

if [ ! -s "$archive" ]; then
    curl -fSL -o "$archive" "https://drive.usercontent.google.com/download?id=1_849LvuT3WBEHktBT97P2oMBzeJz7-UP&export=download&confirm=t"
fi
got=$(sha256sum "$archive" | cut -d' ' -f1)
if [ "$got" != "$want" ]; then
    echo "test_set.tar.bz2: SHA-256 $got, expected $want" >&2
    exit 1
fi
tar -xjf "$archive" -C "$dir"
echo "SAN's test set is in $dir/Mixamo"
