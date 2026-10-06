# G1/data

    recordings/    ban ghi rt/lowstate THO (.npz) - KHONG vao git, nhung o day
                   chu khong o /tmp (da tung mat sach du lieu vi de /tmp)
    test_vectors/  file chuan de doi chieu ban C++ (.txt/.json) - CO vao git,
                   day la moc so sanh, mat la phai co robot moi tao lai duoc

## Ghi (can robot, chi doc, khong gui lenh)

    source G1/src/g1_leg_odometry/scripts/env.sh
    ros2 launch g1_wbc record.launch.py label:="day vai + di vai buoc" secs:=40

Danh dau su kien trong luc ghi, tu terminal khac:

    ros2 topic pub --once /rec_mark std_msgs/String "{data: 'day lan 1'}"

## Dung lai (khong can robot)

    .venv-real/bin/python G1/src/g1_wbc/g1_wbc/replay.py G1/data/recordings/X.npz
    ... --sweep kd_com=0,15,30
    .venv-real/bin/python G1/src/g1_wbc/g1_wbc/test_vectors.py make \
        G1/data/recordings/X.npz -o G1/data/test_vectors/v1 -n 40
    .venv-real/bin/python G1/src/g1_wbc/g1_wbc/test_vectors.py check \
        G1/data/test_vectors/v1.txt ra_cpp.txt
