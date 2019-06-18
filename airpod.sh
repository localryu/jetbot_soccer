NET=/home/ryu/jetson-inference/data/networks/detectnet-airpod

./detectnet-camera --prototxt=$NET/deploy.prototxt --model=$NET/snapshot_iter_1152.caffemodel --mean_binary=$NET/mean.binaryproto --input_blob=data --ouput_cvg=coverage --output_bbox=bboxes --class_labels=$NET/class_labels.txt
