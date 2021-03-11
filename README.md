# Delivery Robot Node 2

## <u>1. Overview</u>
&nbsp; Delivery Robot Node 2ソフトウェアは、2019年度に開発した「複数ロボットによる建物内自動搬送を目的とした2Dナビゲーションソフトウェア」の実証実験にて顕在化した、課題を改善するために機能の追加実装を行ったソフトウェアとなります。<br>
ROSのノードとして機能します。<br>
事前にROSのnavigation_stack（amcl+move_base）環境を準備、各ソフトウェアをリリース後起動して、所定の書式に則った移動指示TOPICを移動指示用ノードから配信することで、目的地への移動を行います。<br>

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/01.png)

本ソフトウェアでは2019年度実証実験の課題として以下2点を解決するために開発されました。<br>
【課題①】ロボット同士の衝突及び干渉の発生<br>
ロボット同士がすれ違う場合、互いの経路情報が不明の為、下記のようにそれぞれのロボットの回避内容にバラツキが発生してしまいます。<br>

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/02.png)

【課題②】複数Way Pointを経由した移動を行う場合の問題<br>
目的地到着までの過程で、人の密集地帯を避けるなど動的に経路の制御を行う場合、複数Way Pointを指定することで実現しておりました。<br>
しかし、ロボットはWay Pointへ到着する度に一時停止し、次のWay Pointの方向に旋回した後移動を再開する為、異常停止しているように見えるだけでなく、目的地到着まで時間が掛かってしまいます。<br>
また、経由するWay Point上に障害物が置かれた場合に、ロボットはそのWPへ到達出来ず、スタックしてしまいます。<br>



上記課題を解決する為に、以下2点の対応を行いました。<br>
（１）観測障害物の動的地図追加<br>
観測した動的障害物情報をGlobalCostmap上に追加することで、事前に観測した障害物を反映した効率的な局所経路計画を行えるようにしました。

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/03.png)

（２）ポテンシャル場を用いた局所経路探索<br>
複数ロボットの運行下において、大局経路地図上で、運行中のロボットの経路を除いたフィールドから通行可能なエリアを谷底とするポテンシャル場を移動指示と共にロボットが受け取り、<br>
ロボットは受け取ったポテンシャル場をコストマップ上に反映し、制限された通行区域内で局所的経路計画を行うようにすることで、ロボット間の衝突や単区間での待ち合わせを回避し、スムーズな移動を可能としました。

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/04.png)


## <u>2. System requirements</u>
### 2.1 Hardware
&nbsp; ロボット：ROS対応台車型ロボット（Turtlebot、メガローバー等）<br>
&nbsp; センサー：2D/3D LiDAR<br>
&nbsp; ロボット制御用PC：Barebone PC推奨<br>
&nbsp; 遠隔監視用PC	：指定なし<br>

【ハードウェア構成例】<br>
![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/05.png)



### 2.2 Software
&nbsp; OS：Ubuntu 16.04 LTS <br>
&nbsp; ROS：Kinetic<br>
&nbsp; Localization：amcl（Navigation_Stack）<br>
&nbsp; Navigation：move_base（Navigation_Stack）<br>
&nbsp; Odometory：Wheel＋LiDAR<br>
 
##<u>3. Software structure</u>
&nbsp; ソフトウェアはROSノードにより構成されます。ノード構成を以下に示します。

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/06.png)

&nbsp; 複数ロボットに対応するため、ロボット毎に[Entity_Type]、[Entity_ID]という識別子を付与します。この識別子を使用して、同一ROSネットワーク内で複数ロボットを制御します。<br>
　　・Entity_Type	…ロボット名（例：turtlebot、megarover）<br>
　　・Entity_ID	…ロボットの通番（例：turtlebot_01、megarover_01）<br>

　ROS NODE/TOPIC/TF名に識別子が入るよう、各種設定ファイルをご変更下さい。<br>
　　・ROS NODE…例：/[Entity_ID]/move_base、/[Entity_ID]/amcl<br>
　　・ROS TOPIC…例：/[Entity_ID]/cmd_vel、/[Entity_ID]/odom<br>
　　・ROS TF…例：[Entity_ID]/baselink、[Entity_ID]/base_footprint<br>
　
　以下にROS NODE/TOPIC/TFの命名例をrqt_graphとtf_treeにて示します。

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/07.png)

![image](https://github.com/jadsys/DeliveryRobotNode2/wiki/images/08.png)

## <u>4. Installations</u>
&nbsp;システム構築手順については  "[Installation Manual](InstallationManual.md)"をご参照下さい。 

## <u>5. How to Use</u>
&nbsp;使用方法については  "[How to use](HowToUse.md)"をご参照下さい。  

## <u>6. License</u>

[BSD](LICENSE)

