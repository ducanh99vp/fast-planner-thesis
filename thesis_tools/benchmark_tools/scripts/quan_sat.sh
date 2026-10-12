#!/usr/bin/env bash
# [CD2] Quan sat tu dong: chay lien tiep nhieu luot, moi luot mot kich ban vat
# can dong khac (hat giong tang dan), UAV tu cat canh va tu nhan dich (19, 0).
# Nguoi dung chi can nhin RViz.
#
# BAT BUOC chay roscore RIENG truoc (xem huong dan). Neu khong, moi luot
# roslaunch tu bat roi tu tat master, RViz mat ket noi va dung hinh sau luot 1.
#
# Dung:
#   ./quan_sat.sh                     # 20 luot, cau hinh mac dinh ben duoi
#   ./quan_sat.sh 5                   # 5 luot
#   SEED0=40 DYN=8 P=30 C=5 ./quan_sat.sh 10
set -u

N=${1:-20}             # so luot
SEED0=${SEED0:-1}      # hat giong luot dau
DYN=${DYN:-8}          # so vat can dong (4 hop + 4 tru)
P=${P:-60}             # cot vuong tinh
C=${C:-10}             # vanh tron tinh
H=${H:-3.0}            # chieu cao thap nhat cua cot (3.0 = khong bay vuot)
Y=${Y:-3.0}            # nua be rong hanh lang sinh vat can dong quanh y = 0
TMO=${TMO:-70}         # gioi han moi luot (giay)
OUT=/tmp/quan_sat.csv  # ghi ra /tmp, khong lan vao results/

if ! rostopic list >/dev/null 2>&1; then
  echo "LOI: chua co roscore. Mo mot terminal va chay:  roscore"
  exit 1
fi

trap 'echo; echo "Dung theo yeu cau."; kill 0' INT

for ((k = 0; k < N; k++)); do
  SEED=$((SEED0 + k))
  echo "=== luot $((k + 1))/$N  |  hat $SEED  |  $DYN vat can dong (|y|<=$Y m), $P cot cao >= $H m, $C vanh ==="
  roslaunch benchmark_tools bench.launch \
    map_name:=QS config:=QS trial:=$k goal_x:=19.0 goal_y:=0.0 timeout:=$TMO \
    use_pcd:=false map_size_x:=40.0 map_size_y:=20.0 map_size_z:=5.0 \
    init_x:=-19.0 init_y:=0.0 init_z:=0.0 takeoff_height:=1.0 \
    max_vel:=1.5 max_acc:=1.5 p_num:=$P c_num:=$C p_hmin:=$H \
    dyn_obs:=1 dyn_num:=$DYN dyn_seed:=$SEED dyn_xy_y:=$Y \
    out_csv:=$OUT enc_csv:=/tmp/quan_sat_enc.csv 2>&1 \
    | grep --line-buffered -E "Cham tran voi vat can dong|Kết thúc|VA CHẠM" \
    | sed -u 's/\x1b\[[0-9;]*m//g; s/^.*\[metric_logger\] /    /'
  sleep 2
done
echo "Xong $N luot. Tom tat: python3 phan_tich_cham_tran.py $OUT"
