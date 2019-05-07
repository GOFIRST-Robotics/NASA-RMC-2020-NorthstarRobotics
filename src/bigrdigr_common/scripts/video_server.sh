#rosrun web_video_server web_video_server

chromium-browser --app --new-window  http://localhost:8080/stream?topic=/usb_cam_pseye0/image_raw
chromium-browser --app --new-window  http://localhost:8080/stream?topic=/usb_cam_pseye1/image_raw
chromium-browser --app --new-window  http://localhost:8080/stream?topic=/usb_cam_pseye2/image_raw
chromium-browser --app --new-window  http://localhost:8080/stream?topic=/usb_cam_pseye3/image_raw
chromium-browser --app --new-window  http://localhost:8080/stream?topic=/usb_cam_pseye4/image_raw
