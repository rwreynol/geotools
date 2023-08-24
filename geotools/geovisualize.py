from osgeo import gdal
from osgeo import osr
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from scipy import io

class geoheatmap(object):
    '''
    '''

    def from_xyz(self,file):
        '''
        '''
        print('Not implemented yet')
        pass

    def from_grid(self,file,lat,lon,rgb,alpha):
        '''
        '''
        self._create_rgb(file,lat,lon,rgb,alpha)

    def _create_rgb(self,name,lat,lon,rgb,alpha):
        '''
        '''
        nx = rgb.shape[1]
        ny = rgb.shape[0]
        xmin, ymin, xmax, ymax = [np.min(lon), np.min(lat), np.max(lon), np.max(lat)]
        xres = (xmax - xmin) / float(nx)
        yres = (ymax - ymin) / float(ny)
        geotransform = (xmin, xres, 0, ymin, 0, yres)
        print('Lat(deg): [%.7f,%.7f] N: %d Res: %f' % (ymin,ymax,ny,yres) )
        print('Lon(deg): [%.7f,%.7f] N: %d Res: %f' % (xmin,xmax,nx,xres) )

        # create the 3-band raster file
        dst_ds = gdal.GetDriverByName('GTiff').Create(name, nx, ny, 4, gdal.GDT_Byte)

        dst_ds.SetGeoTransform(geotransform)    # specify coords
        srs = osr.SpatialReference()            # establish encoding
        srs.ImportFromEPSG(4326)                # WGS84 lat/long 4326
        dst_ds.SetProjection(srs.ExportToWkt()) # export coords to file
        dst_ds.GetRasterBand(1).WriteArray(rgb[:,:,0])   # write r-band to the raster
        dst_ds.GetRasterBand(2).WriteArray(rgb[:,:,1])   # write g-band to the raster
        dst_ds.GetRasterBand(3).WriteArray(rgb[:,:,2])   # write b-band to the raster
        dst_ds.GetRasterBand(4).WriteArray(alpha)   # write b-band to the raster
        dst_ds.FlushCache()                     # write to disk
        dst_ds = None

def main():
    '''
    '''
    mat = io.loadmat('ivs.mat')
    rgb = mat['rgb_image']
    c   = mat['cdata']
    a   = mat['adata'] * 255
    lat = mat['lat']
    lon = mat['lon']

    map = geoheatmap()
    map.from_grid('test.tiff',lat,lon,rgb,a)

if __name__ == '__main__':
    '''
    '''
    main()

