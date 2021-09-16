

class ObjParts:
    """
    ObjParts Class, used to help storing data from the point cloud.

    Parameters:
    -----------
        cloud: pcl.PointCloud()
        altura: float
        largura: float
        curvatura: float
        delta: float
        id: int
    """
    def __init__(self, cloud, altura=0, largura=0, curvatura=0, delta=0, id =0):
        self.cloud = cloud
        self.altura = altura
        self.largura = largura
        self.curvatura = curvatura
        self.delta = delta
        self.id = id
    def __str__(self):
        output = "Height: " + str(self.altura) + " Width: " + str(self.largura) + " Curvature: " + str(self.curvatura) + " Delta: "+ str(self.delta) +" id: " + str(self.id)
        return output