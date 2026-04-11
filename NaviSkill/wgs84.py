import os
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

class WGS84Converter:
    def __init__(self):
        self.origin_lon = None
        self.origin_lat = None
        self.yaw = None  # 地图坐标系相对于经纬度坐标系的朝向（弧度）
        self.data = None

    def init_by_map(self, map_file="config/map.txt"):
        """
        从map.txt读取原点和朝向
        文件格式: origin_lon,origin_lat,yaw
        """
        if not os.path.exists(map_file):
            print(f"地图文件{map_file}不存在")
            return False

        try:
            with open(map_file, "r", encoding="utf-8") as f:
                line = f.readline().strip()
                if not line:
                    print(f"地图文件{map_file}为空")
                    return False
                
                # 解析格式: lon0,lat0,theta
                parts = line.split(',')
                if len(parts) != 3:
                    print(f"地图文件格式错误，期望3个值，实际{len(parts)}个")
                    return False
                
                # 读取3个值：经度，纬度，朝向
                lon0, lat0, theta = map(float, parts)
                
                # 设置类属性
                self.origin_lon = lon0
                self.origin_lat = lat0
                self.yaw = theta
                
                print(f"已从{map_file}读取地图原点和朝向:")
                print(f"  原点经度: {self.origin_lon}")
                print(f"  原点纬度: {self.origin_lat}")
                print(f"  朝向角度: {self.yaw} 弧度 ({self.yaw*180/np.pi:.4f}度)")
                
                return True
            
        except FileNotFoundError:
            print(f"地图文件{map_file}不存在")
            return False
        except ValueError as e:
            print(f"解析地图文件{map_file}出错，请检查数值格式: {e}")
            return False
        except Exception as e:
            print(f"读取地图文件{map_file}时发生未知错误: {e}")
            return False

    def save_map(self, lat0, lon0, theta, map_file="config/map.txt"):
        """
        保存原点和朝向到map.txt
        """
        with open(map_file, "w", encoding="utf-8") as f:
            f.write(f"{lon0},{lat0},{theta}\n")
        print(f"已保存地图原点和朝向到{map_file}")
        
    def init_by_traj(self, traj_file="config/traj.txt"):
        self.data = self.read_data(traj_file)
        # 优化目标：使xy原点(0,0)在经纬度系的lat0,lon0,theta下误差最小
        init = [self.data[0,2], self.data[0,3], 0]  # 初始猜测
        res = minimize(
            self.total_error,
            init,
            args=(self.data,),
            method='Powell',
            callback=self.cb,
            options={'disp': True}
        )
        lat0, lon0, theta = res.x
        print(f"最优原点(0,0)在经纬度: lat={lat0}, lon={lon0}, x轴与正北夹角={theta*180/np.pi:.4f}度")
        self.save_map(lat0, lon0, theta=theta)
        self.origin_lat = lat0
        self.origin_lon = lon0
        self.yaw = theta

        # self.plot()
        return True



    def initialize(self):
        """
        优先用map.txt初始化，否则用traj.txt
        """
        if not self.init_by_map():
            self.init_by_traj()

    def lonlat_to_xy(self, lon, lat):
        """
        经纬度转map坐标系
        """
        # 粗略球面近似，适合小范围
        return self.latlon_to_xy(lat, lon, self.origin_lat, self.origin_lon, self.yaw)

    def xy_to_lonlat(self, x, y):
        """
        map坐标系转经纬度
        """
        lat, lon = self.xy_to_latlon(x, y, self.origin_lat, self.origin_lon, self.yaw)
        return lon, lat

    def read_data(self, filename):
        data = []
        with open(filename) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 5:
                    continue
                x, y, lat, lon, t = map(float, parts[:5])
                data.append((x, y, lat, lon))
        return np.array(data)

    def latlon_to_xy(self, lat, lon, lat0, lon0, theta):
        R = 6378137
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        lat0_rad = np.radians(lat0)
        lon0_rad = np.radians(lon0)
        dx = (lon_rad - lon0_rad) * np.cos(lat0_rad) * R
        dy = (lat_rad - lat0_rad) * R
        # 旋转theta（x轴与正北夹角，正为逆时针）
        x = dx * np.cos(theta) + dy * np.sin(theta)
        y = -dx * np.sin(theta) + dy * np.cos(theta)
        return x, y

    def xy_to_latlon(self, x, y, lat0, lon0, theta):
        R = 6378137
        # 逆旋转
        dx = x * np.cos(theta) - y * np.sin(theta)
        dy = x * np.sin(theta) + y * np.cos(theta)
        lat = lat0 + (dy / R) * 180 / np.pi
        lon = lon0 + (dx / (R * np.cos(np.radians(lat0)))) * 180 / np.pi
        return lat, lon

    def total_error(self, params, data):
        lat0, lon0, theta = params
        x, y, lat, lon = data[:,0], data[:,1], data[:,2], data[:,3]
        # 经纬度->xy误差
        x1, y1 = self.latlon_to_xy(lat, lon, lat0, lon0, theta)
        err1 = np.sqrt((x1-x)**2 + (y1-y)**2)
        # xy->经纬度误差（单位m）
        lat2, lon2 = self.xy_to_latlon(x, y, lat0, lon0, theta)
        err2 = np.sqrt(((lat2-lat)*111000)**2 + ((lon2-lon)*111000*np.cos(np.radians(lat0)))**2)
        return np.mean(err1) + np.mean(err2)

    def cb(self, params):
        err = self.total_error(params, self.data)
        print(f"迭代参数: lat0={params[0]:.8f}, lon0={params[1]:.8f}, theta={params[2]:.6f} 残差={err:.6f}")

    def plot(self):
        if self.data is None:
            print("没有数据可供绘图")
            return
        x, y, lat, lon = self.data[:,0], self.data[:,1], self.data[:,2], self.data[:,3]
        x1, y1 = self.latlon_to_xy(lat, lon, self.origin_lat, self.origin_lon, self.yaw)
        plt.figure()
        plt.plot(x, y, 'r.', label='xy')
        plt.plot(x1, y1, 'b.', label='lonlat->xy')
        plt.axis('equal')
        plt.legend()
        plt.title('xy vs lonlat->xy')
        plt.show()