# How to Use 

&nbsp; 本プログラムを使用したナビゲーションを行います。<br>
※事前にgmappinng等で2D占有格子地図を作成しておき、ファイルの格納と、Navigation_Stackへの設定を行って下さい。

#### （１）ロボット設定
・ロボットのナビゲーション時の挙動に関わる設定を編集します。<br>
以下コマンドを入力し、ファイルを編集します。既定値でも問題なく動作するため、必要に応じて変更して下さい。<br>
[ロボット統括ノード]
```bash
$ cd ~/catkin_ws/src/delivery_robot/param
$ vi delivery_robot_node_tb3_01_L.yaml
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/15.png) 

　編集可能なパラメータを以下に記述します。
```bash
No.3「navigation_turn_speed」は、ナビゲーション開始時に機体を目的地点方向へ向ける際の旋回速度及び、目的地点に到達した際に指定された向きへロボットを旋回させる速度を示します。
No.4「wp_sleep_time」は、次のWPに移動を開始するまでの時間を示します。
No.5「goal_tolerance_range」は、目標地点座標の誤差の範囲を示します。厳密にする場合は小さい値を、ある程度の誤差を許容する場合は大きな値を設定して下さい。
No.6「goal_allowable_range」は、ロボット停止タイマーを開始する範囲を指定します。目標地点付近で、誤差の影響でロボットが止まらない場合の対応として、タイマーを発行し、タイマーT.O.が発生したらロボットは目標地点に到達したと見なし、ロボットを停止させます。
No.7「goal_allowable_time」は、ロボット停止タイマーの時間を指定します。用途はNo.6と同様です。
No.8「goal_allowable_angle」は、目標地点の向きの誤差の範囲を示します。厳密にする場合は小さい値を、ある程度の誤差を許容する場合は大きな値を設定して下さい。
No.9「stuck_check_time」は、ロボットがスタックしていないかチェックする時間間隔を示します。値を小さくすることで、ロボットがスタックした場合に復帰までの時間が短くなりますが、短くしすぎると動作に影響を及ぼす可能性があります。移動速度[m/s]×stuck_check_time[s]がstuck_threshold_length[m]以下にならないようにする必要があります。適正値は5~10秒です。
No.10「stuck_threshold_length」は、スタックしたと判定する際の移動距離の閾値を示します。厳密にする場合は値を小さくします。なお、値はロボットの移動速度[m/s] × stuck_check_time[s]以上の値にならないようにする必要があります。適正値は5cm~10cm程度です。
No.11「footprint」は、ロボットのfootprint情報を示します。ロボットの情報として上位側へ通知します。
No.12「robot_radius」は、ロボットの半径の大きさを示します。ロボットの情報として上位側へ通知します。
No.13「inflation_raidus」は、ロボットのマージン込みの半径の大きさを示します。ロボットの情報として上位側へ通知します。
No.14「turn_control_exclusion_range」は、ナビゲーション開始時に目的地点へ向けた旋回処理を行わない範囲を示します。LICTiAの会議室1~3の入口など、ロボットが旋回することが難しい範囲をマップ上の座標で設定します。
```

[地図更新ノード]
```bash
$ cd ~/catkin_ws/src/map_controller/param
$ vi map_organizer_node_turtlebot_01.yaml
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/16.png) 

 　編集可能なパラメータを以下に記述します。
```bash
No.1「costmap_updates_wait_count」は、/costmap_updates受信時に1インクリメントされ、カウンタ数に到達した際にローカルに保持するコストマップに反映します。
No.3「publish_cycle_count」は、プログラムループ毎に1インクリメントされ、カウンタ数に到達した際に地図を配信します。
No.4「cost_output_count」は、ソシオ地図（2D格子地図）配信間隔毎に1デクリメントされ、0に到達した際にローカルに保持するソシオ地図からコストを削除します。
No.5「cost_existence_check_count」は、ソシオ地図配信間隔毎にコストが存在する場合に1デクリメントされ、に到達した際にローカルに保持するソシオ地図にコストを反映します。
```

[経路探索ノード]
```bash
$ cd ~/catkin_ws/src/map_controller/param
$ vi map_organizer_node_turtlebot_01.yaml
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/17.png) 

 　編集可能なパラメータを以下に記述します。
```bash
No.1「entity_id」は、ロボット毎にロボット制御統括ノードから配信される経路コストマップのトピック名（/Entity_ID/plan_costmap）の指定に使用します。
```


#### （２）ロボットの電源を入れる
・ロボット、センサー、PCの電源を入れて下さい。

#### （３）ロボットの移動
・付属のコントローラで、地図の初期位置にロボットを移動させます。
 
#### （４）ターミナルの準備
・ナビゲーションを行うには、ロボット1台につき、計9個のターミナルにてコマンド入力を行う必要があります。<br>
①～⑤については、状態表示用PCからロボット制御用PCにssh接続します。
```bash
$ ssh azu001@[ロボット制御用PCのIPアドレス]
```
```bash
【ロボット制御用PCにssh接続するターミナル】
　　ターミナル①：ロボット制御ノード起動用
　　ターミナル②：Velodyneドライバノード起動用
　　ターミナル③：move_baseノード起動用
　　ターミナル④：地図更新ノード起動用
　　ターミナル⑤：ロボット制御統括ノード起動用
```
```bash
【状態表示用PCで起動するターミナル】
　　ターミナル⑥：状態表示（rviz）ノード起動用
　　ターミナル⑦：緊急停止コマンド配信用
　　ターミナル⑧：経路コストマップ配信ノード起動用 
　　ターミナル⑨：移動指示用ノード起動用

```

#### （５）ロボット側プログラムの起動
・（４）にて起動したターミナル上で以下のコマンドを入力し、プログラムを順次起動します。<br>
⑥は地図更新の状態を確認するため④の前に起動します。<br>
コマンド投入後のターミナルの画面を以下に添付致しますので、同じメッセージが表示されることをご確認下さい。<br>
【ターミナル①】<br>
```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/18.png) 
 
 
【ターミナル②】<br>
```bash
$ roslaunch velodyne_pointcloud VLP16_points.launch
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/19.png) 

【ターミナル③】<br>
```bash
$ roslaunch turtlebot3_navigation turtlebot3_navigation_L.launch
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/20.png) 
 
【ターミナル⑥】※④の前に起動します。
```bash
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
$ rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_navigation.rviz
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/21.png) 

【ターミナル④】<br>
```bash
$ roslaunch map_controller map_organizer_tb3_01.launch
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/22.png) 

地図更新ノードが起動するとlocalとglobalな地図を比較し、差分箇所を進入禁止区域として上書きし、格子地図を再配信します。<br>
rviz画面上で地図とロボットの位置にズレが生じず、赤色で進入禁止区域が上書きされることを確認します。<br>
万が一ズレが生じてしまった場合には、 ③からやり直す必要があります。

【ターミナル⑤】<br>
```bash
$ roslaunch delivery_robot delivery_manager_tb3_01_L.launch
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/23.png) 
 
ロボット制御統括ノードが起動するとキャリブレーション（360~450°旋回）を行います。<br>
⑥で起動したrviz画面上で、ロボット付近の緑色の点群（パーティクル）が収束していることを確認します。<br>
「Standby…」メッセージの出力を以って、ナビゲーション準備完了となります。

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/24.png) 
 
【ターミナル⑦】<br>
　緊急停止コマンド投入用のターミナルであるため、事前コマンドのみ入力しておきます。<br>
 （緊急停止コマンドはターミナルに貼り付けておき、有事の際に「enter」ボタンを押下するだけにしておくとよろしいかと思います）

```bash
【事前コマンド】
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
```
```bash

【緊急停止コマンド】
【Turtlebot3の場合】
$ rostopic pub -r 10 /turtlebot_01/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

 #### （６）経路コストマップ配信ノードの起動
・Remote PC Setup－（８）にて作成した経路コストマップを使用して通行可能エリアを制限するための、経路コストマップの配信ノードを起動します。<br>
⑨で起動する移動指示用ノードでは本ノードから配信された経路コストマップを受信し、ロボット統括ノードに経路コストマップを含めた移動指示メッセージを送信します。<br>
そのため，通行可能エリアを制限しない場合でも，移動指示メッセージに経路コストマップ情報が必要となります．未作成の場合はRemote PC Setup－（８）の手順に従い，作成して下さい。<br>

【ターミナル⑧】
```bash
【事前コマンド】
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
```
```bash
【ノード起動コマンド】
【Turtlebot3の場合】
$ ROS_NAMESPACE=turtlebot_01 roslaunch turtlebot3_navigation map_server_costmap_L.launch
```

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/25png) 

経路コストマップ配信ノードプログラムを終了したい場合には「Ctrl + C」を押下します。

#### （７）移動指示用ノードの起動
・ナビゲーションを実行するための移動指示用ノード起動用のターミナル⑨を起動します。

【ターミナル⑨】
```bash
【事前コマンド】
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
```
・Remote PC Setup－（９）の項でプログラム修正した経路パターンを指定し、移動指示用ノードを起動します。
```bash
【ノード起動コマンド】
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 154  // navi
```

　コマンドを入力すると以下のようなメッセージが出力され、ロボットにメッセージをTOPIC配信し、ロボットがナビゲーションを開始します。<br>
 

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/26.png) 

 

ロボットからの移動指示結果応答コマンドを受け取ると上記赤枠のメッセージを表示し、移動指示用ノードが終了します。<br>
再度ナビゲーションを行う場合は、別途navi用のコマンドを入力して下さい。<br>

#### （８）ナビゲーション
・移動指示用ノード用のターミナルから、navi用のコマンドを実行し、ロボットにナビゲーションを指示します。<br>
ロボットが移動指示メッセージを受信するとターミナル⑤に「Applying goal」のメッセージが出力され、ナビゲーションを開始します。<br>

【ナビゲーション開始時のターミナル⑤の出力】<br>

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/27.png) 

【ナビゲーション中のrvizの表示】<br>
［通行経路を制限しない場合］
![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/28.png) 

［通行経路を制限している場合］
![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/29.png) 

目標地点は赤い矢印、経路は線で表示されます。<br>

ロボットが目的地点に到着すると、ターミナル⑤に「way point Goal」のメッセージが出力され、ロボットが停止し、次の移動指示待ちの状態となります。<br>

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/30.png) 


#### （９）プログラムの終了
・ナビゲーションを終了する場合、またはバッテリー交換等で作業を終了する場合は、各プログラムを終了させます。<br>
⑨→①の順に「Ctrl + C」を押下し、プログラムを終了します。<br>
ロボット制御用PCにssh接続したターミナルで、以下コマンドを入力し、ロボット制御用PCをシャットダウンします。<br>

```bash
$ sudo shutdown –h now
```

#### （１０）ロボットの電源を切る
・PC、センサー、ロボットの電源を順次切ります。<br>


# Recovery procedure
&nbsp; ロボットが自己位置を失った際のリカバリ手順について、以下に記述します。

#### （１）	緊急停止
・予め起動してあるターミナル⑦にて緊急停止用コマンドを入力します。

・緊急停止用コマンドを受信するとロボットは微動状態になりますので、次にターミナル③上で「Ctrl + C」を押下し、move_baseノードを停止します。

・move_baseノード停止により、ロボットは動作を停止します。<br>
ロボットの停止確認後、ターミナル⑨、⑧、⑤、④の順に「Ctrl + C」を押下し、移動指示用ノード、経路コストマップ配信ノード、ロボット制御統括ノード、地図更新ノードを停止します。

　※伝送遅延やタイミングの問題で、停止が遅れる場合がございますため、オペレーターとは別の要員が、ロボットが壁や障害物にぶつからないようにロボットを持ち上げる、障害物とロボットの間に体を差し込むといった対応を取られた方が良いです。

#### （２）リカバリ
・リカバリはプログラムの再起動となります。まずロボットを初期位置に移動させます。<br>
　（手動搬送、コントローラによる移動、どちらでも問題ありません）

・プログラムをターミナル上で順次「Ctrl + C」を押下して停止させます。

・停止させた後、ターミナル①から順次プログラムを起動します。
　キャリブレーションのためロボットが旋回行動を取りますので、誤動作と間違われないようお願い致します）
