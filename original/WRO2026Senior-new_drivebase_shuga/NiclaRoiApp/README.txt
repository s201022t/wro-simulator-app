NiclaRoiApp 作成手順
=========================

1. このフォルダーをPCへコピー
   例:
   C:\Users\progl\NiclaRoiApp

2. install_requirements.bat をダブルクリック
   PySide6 / pyserial / Pillow / NumPy / PyInstallerをインストールします。

3. Nicla VisionをUSB接続
   Arduino IDEのシリアルモニターと旧Pythonアプリは閉じます。

4. run_app.bat をダブルクリック
   COMポートを選び「接続」を押します。

5. 動作確認
   - 新しく撮影して表示
   - 直近の保存画像を表示
   - NiclaのROI設定を取得
   - 12個のROIをNiclaへ送信
   - 色読みテスト
   - HSV設定の取得・送信

6. まず通常形式のEXEを作る
   build_test.bat をダブルクリック

   生成場所:
   dist\NiclaRoiApp\NiclaRoiApp.exe

7. 通常形式が動いたら単一EXEを作る
   build_exe.bat をダブルクリック

   生成場所:
   dist\NiclaRoiApp.exe

注意
----
- USBシリアルは同時に1つのソフトしか使用できません。
- Nicla側には nicla_motion_camera_v3.ino を書き込んでください。
- MOTIONからREADした画像は「直近の保存画像を表示」で取得します。
- 画像は次にも自動保存されます:
  C:\Users\<ユーザー名>\Documents\NiclaRoiApp\nicla_latest.png
- ROI設定とHSV設定は現在NiclaのRAM保持です。
  Niclaをリセットまたは電源OFFすると初期値へ戻ります。
