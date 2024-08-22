# debug_package

AIチャレンジ2024のAWSIM,Autowareから得られるデータを確認するためのパッケージを集めたプロジェクトになります。

## setup
```shell
#AIチャレンジディレクトリ内にクローンことをおすすめします
cd aichallenge-2024/aichallenge/workspace/src/aichallenge_submit/
git clone https://github.com/Arata-Stu/debug_package.git

#docker内で
cd /aichallenge/workspace
colcon build --symlink-install 
```

## run
```shell
cd /aichallenge/workspace
source install/setup.bash
ros2 launch debug_launch debug.launch.xml
```