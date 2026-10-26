#!/usr/bin/env python3
"""Real YOLO inference on an official sample image, metric geometry with synthetic depth.
No ROS or hardware. Synthetic depth is not a measured object position.
"""
import json
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO
from graspgen_vision.geometry import object_points,pca_grasp

torch.set_num_threads(4)
root=Path(__file__).resolve().parents[2]
image=cv2.imread(str(root/'.cache/vision-bus.jpg'))
assert image is not None,'Download official sample to .cache/vision-bus.jpg'
image=cv2.resize(image,(640,480))
model=YOLO(str(root/'models/yolov8n-seg.pt'))
r=model.predict(image,device='cpu',retina_masks=True,verbose=False)[0]
assert r.masks is not None and len(r.boxes)>0,'Expected real sample detections'
cv2.imwrite(str(root/'.cache/vision-sample-segmentation.jpg'),r.plot())
depth=np.full((480,640),.35,np.float32)
k=np.array([[700.,0,320.],[0,700.,240.],[0,0,1.]])
records=[]
for idx,mask in enumerate(r.masks.data.cpu().numpy()):
 record={'class':r.names[int(r.boxes.cls[idx])],'score':float(r.boxes.conf[idx])}
 try:
  p,rgb=object_points(mask>.5,depth,image,k)
  record['points']=len(p)
  grasp=pca_grasp(p)
  record.update(candidate=True,opening_m=grasp['opening'],center=grasp['center'].tolist())
 except ValueError as e:record.update(candidate=False,reason=str(e))
 records.append(record)
assert any(v.get('candidate') for v in records),'Expected at least one candidate in synthetic-depth sample'
print(json.dumps({'source':'official sample RGB; SYNTHETIC depth=.35 m','objects':records},indent=2))
print('PASS: real YOLO masks -> filtered RGB object clouds -> PCA candidates (synthetic depth only)')
