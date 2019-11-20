import cv2
import numpy as np

######################################################################
# GLOBAIS
######################################################################
# limiares de saturacao e valor
MINSAT = 60
MAXSAT = 255
MINVAL = 40
MAXVAL = 255

# limiares da cor laranja (TUBO)
MINSATORANGE = 120
ORANGE = 15
DORANGE = 10
MINORANGE = (ORANGE - DORANGE)%180
MAXORANGE = (ORANGE + DORANGE)%180

# limiares da cor vermelha
RED = 170
DRED = 10
MINRED = (RED - DRED)%180
MAXRED = (RED + DRED)%180

# limiares da cor verde
GREEN = 90
DGREEN = 30
MINGREEN = (GREEN - DGREEN)%180
MAXGREEN = (GREEN + DGREEN)%180

# parametros de filtros
GAUSSIAN_FILTER = 3
KERNEL_RESOLUTION = 7

# dimensoes da base real
ARESTA = 50.0 # aresta da base (em mm)
RESOLUTION = 20
	
#############################################################################
# classe para detectar landmarks para o desafio Petrobras
#############################################################################
class Sensors:
	#########################################################################
	# construtor
	#########################################################################
	def __init__(self):
	
		# cria pontos do objeto 3D para o PNP
		self.model3DRect()
		
		# reference frame
		self.rot_vec = np.zeros((4,1))
		self.trans_vec = np.zeros((3,1))
		
		# posicao relativa
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
	
	######################################################################
	# Camera internals
	######################################################################
	def CamParam(self, img):
		
		# tamanho da imagem
		self.size = img.shape
		
		# distancia focal
		self.focal_length = self.size[1]
		
		# centro otico
		self.center = (self.size[1]/2, self.size[0]/2)
		
		# Matriz da camera
		self.camera_matrix = np.array(	[[self.focal_length, 0, self.center[0]],
		                     		[0, self.focal_length, self.center[1]],
		                     		[0, 0, 1]], dtype = "double")
		                     		
		# distorcoes da lente
		self.dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
		#print "Camera Matrix :\n {0}".format(camera_matrix)
		
	######################################################################
	# fornece imagem a ser processada. Fator redimensiona a image
	######################################################################
	def setImage(self, img, fator = 0):
	
		# filtro gaussiano
		img = cv2.GaussianBlur(img, (GAUSSIAN_FILTER, GAUSSIAN_FILTER), 0)
	
		# resize image
		if fator != 0:
			img = cv2.resize(img, None, fx = fator, fy = fator)
		
		# set camera internals from img	
		self.CamParam(img)
		
		# image a ser processada
		self.img = img
	
	######################################################################
	# processamento da imagem, antes do calculo do frame
	######################################################################
	def processImage(self):
		
		# convert self.img para HSV
		hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
		
		# kernel de convolucao
		kernel = np.ones((KERNEL_RESOLUTION, KERNEL_RESOLUTION), np.uint8)
		
		# pega o tubo laranja
		tubo = self.imlimiares(hsv, (MINORANGE, MINSATORANGE, MINVAL), (MAXORANGE, MAXSAT, MAXVAL))
		hsv = cv2.bitwise_and(hsv, hsv, mask = tubo)
		
		# pega o marcador com defeito
		marcador_ruim = self.imlimiares(hsv, (MINRED, MINSAT, MINVAL), (MAXRED, MAXSAT, MAXVAL))
		
		# pega o marcador bom
		marcador_bom = self.imlimiares(hsv, (MINGREEN, MINSAT, MINVAL), (MAXGREEN, MAXSAT, MAXVAL))
		
		# image final
		self.tube = tubo.copy()
		self.bad_mark = marcador_ruim.copy()
		self.good_mark = marcador_bom.copy()
		#
		self.bad_mark = cv2.bitwise_and(self.tube, self.bad_mark, mask = None)
		self.good_mark = cv2.bitwise_and(self.tube, self.good_mark, mask = None)
		
		self.bad_mark = self.imfill(self.bad_mark)
		self.good_mark = self.imfill(self.good_mark)
		
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNEL_RESOLUTION, KERNEL_RESOLUTION))
		self.bad_mark = cv2.morphologyEx(self.bad_mark, cv2.MORPH_OPEN, kernel, iterations = 2)
		self.good_mark = cv2.morphologyEx(self.good_mark, cv2.MORPH_OPEN, kernel, iterations = 2)
	
	######################################################################
	# get reference frame
	######################################################################
	def getRefFrame(self, final_img, tag):
	
		#######################
		# get visual information
		#######################
		# para cada contorno (marcador)
		_, contours, hier = cv2.findContours(final_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		# sucesso do calculo do frame
		success = False
		
		# para todos os contornos encontrados
		for cnt in contours:
			# get convex hull
			hull = cv2.convexHull(cnt)
			
			tadrentro  = True
			for p in hull:
				if self.tube[p[0][1], p[0][0]] == 0:
					tadrentro = False
			if not tadrentro:
				continue
				
			area = cv2.contourArea(cnt)
			if area > 20000:
				continue
			#print area
			
			#cv2.drawContours(self.img, [hull], -1, (0, 0, 255), 3)
			
			try:
				# Fit rectangle or ellipse
				rotrect = cv2.minAreaRect(hull)				
				box = cv2.boxPoints(rotrect)
				box = np.int0(box)
				cv2.drawContours(self.img, [box], 0, (255,255,255), 2)
				cv2.putText(self.img, tag, (box[0][0], box[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))
				
				# calcula projecao: pega pontos da imagem
				self.image2Drect(rotrect)
					
			except:
				#print "Erro ao detectar shape"
				continue
					
			try:
				# calcula projecao
				(success, self.rot_vec, self.trans_vec) = cv2.solvePnP(self.model_points, self.image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
				#print "Rotation Vector:\n {0}".format(rot_vec)
				#print "Translation Vector:\n {0}".format(trans_vec)
			
				# projetando os eixos (25cm)
				if success:
					axis_len = 100.0
					points = np.float32([[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len], [0, 0, 0]]).reshape(-1, 3)
					axisPoints, _ = cv2.projectPoints(points, self.rot_vec, self.trans_vec, self.camera_matrix, self.dist_coeffs)
					#self.img = cv2.line(self.img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
					#self.img = cv2.line(self.img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
					#self.img = cv2.line(self.img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
			except:
				#print "Erro calculando projecao"
				None
		
		return self.rot_vec, self.trans_vec, success
	
	######################################################################
	# get reference frame
	######################################################################
	def getRefFrameSMarks(self):
		
		#######################
		# image processing
		#######################
		self.processImage()
		
		# get bad marks
		R1, T1, success1 = self.getRefFrame(self.bad_mark, "_BAD")
		
		# get good marks
		R2, T2, success2 = self.getRefFrame(self.good_mark, "_OK")
		
		#return R1, T1, success1, R2, T2, success2
		return R1, T1, success1, R2, T2, success2
	
	######################################################################
	# 3D model points. RECTANGLE
	######################################################################
	def model3DRect(self):
		self.model_points = list([])
		self.model_points.append(list([0.0, 0.0, 0.0]))
		self.model_points.append(list([-ARESTA/2.0, -ARESTA/2.0, 0.0]))
		self.model_points.append(list([-ARESTA/2.0, 0.0, 0.0]))
		self.model_points.append(list([-ARESTA/2.0, ARESTA/2.0, 0.0]))
		self.model_points.append(list([0.0, ARESTA/2.0, 0.0]))
		self.model_points.append(list([ARESTA/2.0, ARESTA/2.0, 0.0]))
		self.model_points.append(list([ARESTA/2.0, 0.0, 0.0]))
		self.model_points.append(list([ARESTA/2.0, -ARESTA/2.0, 0.0]))
		self.model_points.append(list([0.0, -ARESTA/2.0, 0.0]))
		
		# retorna np_array
		self.model_points = np.array(self.model_points, dtype=np.float32)
		
	######################################################################
	# 2D model points. RECTANGLE
	######################################################################
	def image2Drect(self, rect):
	
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		
		x = list([])
		y = list([])
		for p in box:
			x.append(int(p[0]))
			y.append(int(p[1]))

		# Center of rectangle in source image
		center = (np.mean(x), np.mean(y))
		
		# lista de pontos da ellipse em 2d
		self.image_points = list([])
	
		# novo ponto do circulo		
		self.image_points.append(list([center[0], center[1]]))
		self.image_points.append(list([x[0], y[0]]))
		self.image_points.append(list([(x[0]+x[1])/2, (y[0]+y[1])/2]))
		self.image_points.append(list([x[1], y[1]]))
		self.image_points.append(list([(x[1]+x[2])/2, (y[1]+y[2])/2]))
		self.image_points.append(list([x[2], y[2]]))
		self.image_points.append(list([(x[2]+x[3])/2, (y[2]+y[3])/2]))
		self.image_points.append(list([x[3], y[3]]))
		self.image_points.append(list([(x[3]+x[0])/2, (y[3]+y[0])/2]))

		# retorna np_array
		self.image_points = np.array(self.image_points, dtype=np.float32)
		
	######################################################################
	# fill holes on images
	######################################################################
	def imfill(self, img):
	
		# kernel de convolucao
		kernel = np.ones((KERNEL_RESOLUTION, KERNEL_RESOLUTION), np.uint8)
	
		# imclose
		img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=3)
	
		# imfill
		_, contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			hull = cv2.convexHull(cnt)
			cv2.drawContours(img, [hull], 0, 255, -1)
				
		return img
		
	######################################################################
	# limiares de cores
	######################################################################
	def imlimiares(self, hsv, hsvMin, hsvMax):
	
		# limiares
		hmin, smin, vmin = hsvMin
		hmax, smax, vmax = hsvMax
		
		# verifica se esta tudo certo	
		if hmin < hmax:
			hsvtresh = cv2.inRange(hsv, hsvMin, hsvMax)
		else:
			hsvtresh1 = cv2.inRange(hsv, (0, smin, vmin), (hmax, smax, vmax))
			hsvtresh2 = cv2.inRange(hsv, (hmin, smin, vmin), (180, smax, vmax))
			hsvtresh = cv2.bitwise_or(hsvtresh1, hsvtresh2, mask = None)
	
		# imfill
		hsvtresh = self.imfill(hsvtresh)
	
		return hsvtresh
	
	#########################################################################
	# show images
	#########################################################################
	def show(self):
	
		m = 1.2
		#cv2.moveWindow('RGB', 1, 1)
		cv2.imshow('RGB', self.img)
		#
		#cv2.moveWindow('Tube', int(m*self.size[1]), 1)
		#cv2.imshow('Tube', self.tube)
		#
		#cv2.moveWindow('Good', 1, int(m*self.size[0]))
		#cv2.imshow('Good', self.good_mark)
		#
		#cv2.moveWindow('Bad', int(m*self.size[1]), int(m*self.size[0]))
		#cv2.imshow('Bad', self.bad_mark)
	
	#########################################################################
	# destrutor
	#########################################################################
	def __del__(self):
		cv2.destroyAllWindows()
