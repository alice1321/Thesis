from pathlib import Path

from tqdm import tqdm
import cv2
import numpy as np
import os
import math
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops, regionprops_table
import matplotlib.patches as patches

data_path = Path('data')
train_path = data_path / 'train'
cropped_train_path = data_path / 'cropped_train'


if __name__ == '__main__':
    for instrument_index in range(1, 9):

        instrument_folder = 'instrument_dataset_' + str(instrument_index)
        (cropped_train_path / instrument_folder / 'labels').mkdir(exist_ok=True, parents=True)

        for file_name in tqdm(list((cropped_train_path / instrument_folder / 'parts_masks').glob('*'))):
            img = cv2.imread(str(file_name))
            name = os.path.splitext(file_name)[0]
            real_image = cv2.imread(str(cropped_train_path / instrument_folder / 'images' / (file_name.stem + '.jpg')))
            height, width, _ = img.shape
            kernel = np.ones((5,5),np.uint8)
            dilation = cv2.dilate(img,kernel,iterations = 5)
            label_img = label(dilation)
            regions = regionprops(label_img)
            #fig, ax = plt.subplots()
            #ax.imshow(real_image, cmap=plt.cm.gray)
            file = open(str(cropped_train_path / instrument_folder / 'labels' /(file_name.stem + '.txt')), 'w')

            for props in regions:
                y0, x0 = props.centroid[0], props.centroid[1]
                #print(x0,y0)
                lab = 0
                minr, minc, maxr, maxc = props.bbox[0], props.bbox[1], props.bbox[3], props.bbox[4]
                w = (maxc-minc)/width
                h = (maxr - minr)/height
                x_center = ((maxc + minc)/2)/width
                y_center = ((maxr + minr)/2)/height
                #print(x_center*width, y_center*height)
                box = [lab, x_center, y_center, w, h]
                i = 0
                for element in box:
                    i +=1
                    '''if element == box[-1]:
                        file.write(str(element) + '\n')
                    else:
                        file.write(str(element) + ' ')'''
                    if i == 5:
                    	file.write(str(element) + '\n')
                    	
                    else:
                    	file.write(str(element) + ' ')
                    	

                #rect = patches.Rectangle(((x_center-w/2)*width, (y_center-h/2)*height), w*width, h*height , linewidth=1, edgecolor='r', facecolor='none')
                #bx = (minc, maxc, maxc, minc, minc)
                #by = (minr, minr, maxr, maxr, minr)
                #ax.plot(bx, by, '-b', linewidth=2.5)
                #ax.add_patch(rect)

            file.close()
            #plt.show()

