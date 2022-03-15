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

./config/speak_list.yamlにトリガーとなるtopicと喋らせたい日本語を設定します。

以下のように実行すると、設定したtopicの日本語をスピーカーが喋ります。

```
roslaunch raspicat_speak raspicat_speak.launch
rostopic pub  -l /hoge2 std_msgs/String "data: 'hogehoge'" 
```