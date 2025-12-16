#!/bin/bash

# supported video extensions
EXT="mp4 mkv avi mov webm"

# check ffprobe is properly working
if ! command -v ffprobe &>/dev/null; then
  echo "Errore: ffprobe non trovato (installa ffmpeg)"
  exit 1
fi

printf "%-45s %-10s %-10s %-12s\n" "FILE" "FRAME" "FPS" "DURATA (ms)"
echo "--------------------------------------------------------------------------------"

# for each video extension
for ext in $EXT; do
  # for each file with the current extension
  for video in *.$ext; do
    # if the video doesn't exist, skip this cycle 
    [ -e "$video" ] || continue

    FPS=$(ffprobe -v error -select_streams v:0 \
      -show_entries stream=r_frame_rate \
      -of default=nokey=1:noprint_wrappers=1 "$video")

    FRAMES=$(ffprobe -v error -select_streams v:0 \
      -show_entries stream=nb_frames \
      -of default=nokey=1:noprint_wrappers=1 "$video")

    # Skip not valid files
    if [[ -z "$FRAMES" || "$FRAMES" == "N/A" ]]; then
      echo "$video  --> frames not available."
      continue
    fi

    # Durata in millisecondi
    DURATA_MS=$(echo "scale=3; ($FRAMES / ($FPS)) * 1000" | bc)

    printf "%-45s %-10s %-10s %-12s\n" "$video" "$FRAMES" "$FPS" "$DURATA_MS"
  done
done
