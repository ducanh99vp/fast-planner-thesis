#!/usr/bin/env bash
# [CD2 Indoor] Quan sat tu dong trong nha: chay lien tiep nhieu luot tren MOT tuyen bay,
# moi luot mot kich ban vat can dong khac (hat giong tang dan). Nguoi dung chi can nhin RViz.
#
# BAT BUOC chay roscore RIENG truoc (xem huong dan), neu khong RViz mat ket noi sau luot 1.
#
# Dung:
#   ./quan_sat_in.sh                          # I3, P2HC (P2 hoan chinh + chong giat + sua yaw), 5 luot
#   MAP=I3G ./quan_sat_in.sh 3                # I3 voi vat can di NGANG hanh lang (can gat nuoc)
#   MAP=I4-R3 ./quan_sat_in.sh 3              # tuyen qua cua 0.9 m, 3 luot
#   MAP=I3 CFG=SW15I2E ./quan_sat_in.sh 3     # ban CHUA chong giat (de so sanh)
#   MAP=I3 CFG=P0 ./quan_sat_in.sh 3          # UAV khong thay vat can dong
#   MAP=I3 SEED0=2 LAP=1 ./quan_sat_in.sh 3   # lap lai hat 2 ba lan
# Tuyen: I1, I3, I4-R1, I4-R2, I4-R3, I5. Cau hinh: xem CONFIGS trong batch_runner.py.
set -u
N=${1:-5}
MAP=${MAP:-I3}
CFG=${CFG:-P2HC}
SEED0=${SEED0:-1}
LAP=${LAP:-0}
TMO=${TMO:-60}
OUT=/tmp/quan_sat_in.csv
cd "$(dirname "$0")"

if ! rostopic list >/dev/null 2>&1; then
  echo "LOI: chua co roscore. Mo mot terminal va chay:  roscore"
  exit 1
fi
trap 'echo; echo "Dung theo yeu cau."; kill 0' INT

for ((k = 0; k < N; k++)); do
  if [ "$LAP" = "1" ]; then SEED=$SEED0; else SEED=$((SEED0 + k)); fi
  echo "=== luot $((k + 1))/$N  |  $MAP  |  $CFG  |  hat $SEED ==="
  python3 batch_runner.py --maps "$MAP" --configs "$CFG" --trials 1 --seed0 "$SEED" --dyn -1 \
    --timeout "$TMO" --out "$OUT" > /tmp/quan_sat_in_run.log 2>&1
  python3 - "$OUT" <<'EOF'
import csv, sys
r = list(csv.DictReader(open(sys.argv[1])))[-1]
kq = "TOI DICH" if r["success"] == "1" else ("VA CHAM TINH" if r["collided"] == "1" else "HET GIO")
print("    %s | %s s | gap vat can dong %s lan, cham %s | nghieng lon nhat %s do | A* that bai %s"
      % (kq, r["T_f"], r["n_enc"], r["n_hit"], r["tilt_max"], r["N_fail"]))
EOF
  sleep 2
done
echo "Xong $N luot. Ket qua tung luot: $OUT"
