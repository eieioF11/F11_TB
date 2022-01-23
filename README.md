# F11_TB
Turtlebot3を使用したアプリケーション
## 環境構築
teb_local_plannerのインストール
```bash
sudo apt-get install ros- melodic -teb-local-planner
rosdep install teb_local_planner
```
## ロボット側で実行するノード
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
カメラを使用する場合
```bash
rosrun cv_camera cv_camera_node _property_0_code:=404 _property_0_code:=1
```
## ロボットの種類指定
```bash
export TURTLEBOT3_MODEL=burger
```
bashrcに上のコマンドを記載すると毎回入力しなくてよくなる。
```bash
gedit ~/.bashrc
```
## キーボード操作
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
## マッピング
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
## ROSナビゲーション
```bash
roslaunch F11_TB navigation.launch map_file:=$HOME/map.yaml
```
## Path Planning & Path Following
```bash
roslaunch F11_TB navi.launch map_file:=$HOME/map.yaml
```
## 全方位カメラを使用した人の検出
### 全方位カメラノードの起動
```bash
roslaunch F11_TB kodak.launch
```
### 人検出ノードの起動
```bash
rosrun F11_TB camera.py
```
![記録画像](/images/mem/0.jpg)
![記録画像](/images/9.jpg)
