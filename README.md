# raspicat_speak
あるROSのtopicがpublishされた時に、任意の日本語をスピーカーで喋らせるためのパッケージです。

## Installation
### Source Build
```
# パッケージのダウンロード
cd ~/catkin_ws/src
git clone https://github.com/uhobeike/raspicat_speak.git

# ビルド＆インストール
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

roscd raspicat_speak/scripts && ./install.sh
```
 
## How to use

[speak_list.yaml](./config/speak_list.yaml)にトリガーとなるtopicと喋らせたい日本語を設定します。

以下の通りに実行すると、speak_list.yamlで設定した日本語をスピーカーが喋ります。

```
roslaunch raspicat_speak raspicat_speak.launch
rostopic pub  -l /hoge2 std_msgs/String "data: 'hogehoge'" 
```