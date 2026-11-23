#!/bin/sh
# Rebuilds dittomo's brand files here: the SVG logos, the retarget frames the cards show, the PNGs and the demo GIF.
#
#     sh assets/brand/build.sh
#
# Needs python, ffmpeg and Chrome or Edge, and the bench rigs (`sh bench/rigs/fetch.sh`). Set BROWSER to the browser's
# path if it isn't found.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
rigs="$root/bench/rigs/data"
frames="$here/src/frames"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp" 2>/dev/null || true' EXIT

python -I "$here/src/brand.py" "$here" >/dev/null

# One samba frame, played from Michelle onto three other bodies.
FRAME=90
(cd "$root" && cargo build -q --release --features fs --example video)
video="$root/target/release/examples/video"
mkdir -p "$frames"
for target in readyplayerme:rpm hvgirl:hvgirl xbot:xbot; do
	file=$(ls "$rigs/${target%%:*}".* | head -1)
	"$video" "$rigs/michelle.glb" "$file" "$tmp/out.mp4" --still $FRAME "$tmp/still.png" --bare 2>/dev/null
	ffmpeg -y -loglevel error -i "$tmp/still.png" -vf crop=480:720:480:0 "$frames/${target##*:}.png"
done
ffmpeg -y -loglevel error -i "$tmp/still.png" -vf crop=480:720:0:0 "$frames/source.png"

# Pages and logos to PNG, through a headless browser.
browser=${BROWSER:-}
for b in "/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
	"/c/Program Files/Google/Chrome/Application/chrome.exe" google-chrome chromium msedge; do
	[ -n "$browser" ] && break
	if [ -x "$b" ] || command -v "$b" >/dev/null 2>&1; then browser=$b; fi
done
url() { if command -v cygpath >/dev/null 2>&1; then echo "file:///$(cygpath -m "$1")"; else echo "file://$1"; fi; }
win() { if command -v cygpath >/dev/null 2>&1; then cygpath -w "$1"; else echo "$1"; fi; }
shots=0
shot() { # page, width, height, out, [scale]
	# A profile of its own each time, as one still closing keeps its profile locked.
	shots=$((shots + 1))
	rm -f "$4"
	"$browser" --headless=new --disable-gpu --hide-scrollbars --no-first-run \
		--user-data-dir="$(win "$tmp/profile$shots")" --allow-file-access-from-files \
		--default-background-color=00000000 --force-device-scale-factor="${5:-1}" \
		--window-size="$2,$3" --screenshot="$(win "$4")" "$(url "$1")" >/dev/null 2>&1 || true
	# The browser can hand off and return before the shot is written.
	n=0
	while [ ! -s "$4" ] && [ $n -lt 100 ]; do sleep 0.2; n=$((n + 1)); done
	[ -s "$4" ] || { echo "no $4" >&2; exit 1; }
	sleep 0.5
}
# An SVG alone on a transparent page, at a size, times a scale.
svgshot() { # svg, width, height, out, scale
	page="$tmp/svg$shots.html"
	echo "<html><body style=\"margin:0;background:transparent\"><img src=\"$(url "$1")\" width=\"$2\" height=\"$3\" style=\"display:block\"></body></html>" >"$page"
	shot "$page" "$2" "$3" "$4" "$5"
}
shot "$here/src/card.html" 1280 640 "$here/social-preview.png"
shot "$here/src/hero.html" 1600 640 "$here/hero.png"
svgshot "$here/dittomo-icon.svg" 512 512 "$here/dittomo-icon-512.png" 1
svgshot "$here/dittomo-icon.svg" 512 512 "$here/dittomo-icon-1024.png" 2
svgshot "$here/dittomo-icon.svg" 64 64 "$here/favicon-64.png" 1
set -- $(sed -n 's/.*viewBox="0 0 \([0-9]*\) \([0-9]*\)".*/\1 \2/p' "$here/dittomo-logo.svg")
svgshot "$here/dittomo-logo.svg" "$1" "$2" "$here/dittomo-logo.png" 2
svgshot "$here/dittomo-logo-light.svg" "$1" "$2" "$here/dittomo-logo-light.png" 2

# The demo: the source and dittomo's copy side by side, five seconds of the samba, kept to a few MB.
"$video" "$rigs/michelle.glb" "$rigs/readyplayerme.glb" "$tmp/demo.mp4" --seconds 5 2>/dev/null
ffmpeg -y -loglevel error -i "$tmp/demo.mp4" -vf "crop=960:720:0:0,fps=12,scale=560:-1:flags=lanczos,split[a][b];\
[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
	"$here/demo.gif"
ls -la "$here"
