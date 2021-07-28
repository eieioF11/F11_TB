# F11_TB
Turtlebot3を使用したアプリケーション
## 環境構築
teb_local_plannerのインストール
```bash
sudo apt-get install ros- melodic -teb-local-planner
rosdep install teb_local_planner
```
## キーボード操作
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
## ナビゲーション
```bash
roslaunch F11_TB navigation.launch map_file:=$HOME/map.yaml
```