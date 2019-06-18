NET=/home/nano/jetson-inference/data/networks/detectnet-airpod

./detectnet-console /home/nano/jetson-inference/data/images/images/test/airpod4.png /home/nano/jetson-inference/data/images/images/test/airpod_out4.png --prototxt=$NET/deploy.prototxt --model=$NET/snapshot_iter_1188.caffemodel --input_blob=data --ouput_cvg=coverage --output_bbox=bboxes --class_labels=$NET/class_labels.txt
