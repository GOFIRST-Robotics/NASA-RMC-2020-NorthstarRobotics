#rosrun web_video_server web_video_server

chromium-browser --new-window --app=http://192.168.1.19:8080/stream?topic=/usb_cam_pseye0/image_raw --window-position=0,0 --window-size=640,480 &
chromium-browser --new-window --app=http://192.168.1.19:8080/stream?topic=/usb_cam_pseye1/image_raw --window-position=1280,0 --window-size=640,480 &
chromium-browser --new-window --app=http://192.168.1.19:8080/stream?topic=/usb_cam_pseye2/image_raw --window-position=0,540 --window-size=640,480 &
chromium-browser --new-window --app=http://192.168.1.19:8080/stream?topic=/usb_cam_pseye3/image_raw --window-position=1280,540 --window-size=640,480 &
chromium-browser --new-window --app=http://192.168.1.19:8080/stream?topic=/usb_cam_pseye4/image_raw --window-position=640,0 --window-size=640,480 &
