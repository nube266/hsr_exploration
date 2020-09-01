# README
## 概要
HSRによる物体探索を行うためのリポジトリ  
[論文](paper/rsj_2020_submit.pdf)  
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

