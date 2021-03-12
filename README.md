# README
## 概要
HSRによる物体探索を行うためのリポジトリ  
./paper以下に資料を追加  
[論文](paper/rsj_2020_submit.pdf)  
[スライド](paper/rsj_presentation.pptx)  
<br/>

## 動作環境
ubuntu 16.04, 18.04  
<br/>

## セットアップ
1.SDEのセットアップ  
```shell
./SETUP-DEVEL-MACHINE.sh
```

2.再起動  
```shell
sudo reboot
```

3.Dockerイメージをビルド  
```shell
./BUILD-DOCKER-IMAGE.sh
```

# 起動方法
dockerコンテナの起動  
```shell
./RUN-TERMINATOR-TERMINAL.sh
```

右上のターミナルで以下のコマンドを入力することでgazebo、rviz、viewpoint_planner_3dが起動  
```shell
roslaunch hsr_launch flexbe_app_default.launch  
```

左から二番目、下から二番目のターミナルで「s」を入力してFlexBEを起動  
```shell
roslaunch hsr_launch flexbe_app_default.launch  
```

FlexBE上でhsr_object_search_simulationを実行する。  

## darknet_rosについて
darknet_rosはgit cloneした段階だと重みファイルがない。  
別途ダウンロードして以下に配置すること。
```bash
hsr_exploration/catkin_ws/src/darknet_ros_one_shoat/darknet_ros/yolo_network_config/weights
```
配布されている重みファイルを使用する場合は、weights/how_to_download_weights.txtを参照。
