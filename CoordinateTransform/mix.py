# import pandas as pd
# import os
# from tqdm import tqdm
# from methods import readPCD,savePCD,xyz2GPS,merge_pcds,plot3D

# pcds_path = '../pcds'
# gps_path = './transfer.csv'
# gps = pd.read_csv(gps_path)

# pcd_list = []

# count_num = 0
# for filename in tqdm(os.listdir(pcds_path)):
#     file_path = os.path.join(pcds_path, filename)
#     if count_num %10 == 0:
#         pcd = readPCD(file_path)
#         u1,u2 = gps.iloc[count_num//10,0],gps.iloc[count_num//10,1]  
#         current_gps = [u1,u2]
#         pcd_gps = xyz2GPS(pcd,current_gps,[0.0,0.0])
#         pcd_list.append(pcd_gps)
#         savePCD(pcd_gps,'./tests/'+str(count_num)+'.pcd')
#     count_num += 1

# pcd = merge_pcds(pcd_list)
# plot3D(pcd)

        




        
