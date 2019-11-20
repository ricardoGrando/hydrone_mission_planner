import cv2
import numpy as np
from imutils.perspective import four_point_transform
import pytesseract
import re

######################################################################
# GLOBAIS
######################################################################
# limiares de saturacao e valor
MINSAT = 50
MAXSAT = 255
MINVAL = 40
MAXVAL = 255

# limiares da cor amarela
YELLOW = 30
DYELLOW = 25
MINYELLOW = YELLOW - DYELLOW
MAXYELLOW = YELLOW + DYELLOW

# limiares da cor azul
BLUE = 110
DBLUE = 25
MINBLUE = BLUE - DBLUE
MAXBLUE = BLUE + DBLUE

# parametros de filtros
GAUSSIAN_FILTER = 3
KERNEL_RESOLUTION = 7

# dimensoes da base real
ARESTA = 500.0 # aresta da base (em mm)
RAIO = 200.0 # raio do centro da base 
RESOLUTION = 50
	
#############################################################################
# classe para detectar landmarks para o desafio Petrobras
#############################################################################
class Digits:
	#########################################################################
	# construtor
	#########################################################################
	def __init__(self):
	
		None
		
	######################################################################
	# fornece imagem a ser processada. Fator redimensiona a image
	######################################################################
	def setImage(self, img, fator = 0):
	
		# filtro gaussiano
		img = cv2.GaussianBlur(img, (GAUSSIAN_FILTER, GAUSSIAN_FILTER), 0)
	
		# resize image
		if fator != 0:
			img = cv2.resize(img, None, fx = fator, fy = fator)
		
		# tamanho da imagem
		self.size = img.shape
		
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
		
		# pega o quadrado azul mais externo
		quadrado_azul = self.imlimiares(hsv, (MINBLUE, MINSAT, MINVAL), (MAXBLUE, MAXSAT, MAXVAL))
		hsv = cv2.bitwise_and(hsv, hsv, mask = quadrado_azul)	
		
		# pega mascadores pretos
		gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
		ret, th1 = cv2.threshold(gray, 40, 255,cv2.THRESH_BINARY_INV)
		self.marcador = th1.copy()
		
		# image final
		self.final = quadrado_azul.copy()
				
		# marcadores finais
		self.final = cv2.bitwise_and(self.final, self.marcador, mask = None)
		self.final = self.imfill(self.final)
		
		#kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (KERNEL_RESOLUTION, KERNEL_RESOLUTION))
		#self.final = cv2.dilate(self.final, kernel)
		#self.final = cv2.morphologyEx(self.final, cv2.MORPH_OPEN, kernel, iterations = 2)
	
	######################################################################
	# get reference frame
	######################################################################
	def getRefFrame(self):
		
		#######################
		# image processing
		#######################
		self.processImage()
		
		#######################
		# get visual information
		#######################
		# para cada contorno (marcador)
		_, contours, hier = cv2.findContours(self.final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		# sucesso do calculo do frame
		success = False
		val1 = '00%'
		val2 = '00%'
		
		displayCnt = None
		
		# loop over the contours
		for c in contours:
			# approximate the contour
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.02 * peri, True)

			# if the contour has four vertices, then we have found
			# the thermostat display
			if len(approx) == 4:
				displayCnt = approx
				break
				
		config = ('-l eng --oem 1 --psm 3 -c tessedit_char_whitelist=0123456789%-')
		try:	
			img = four_point_transform(self.img, displayCnt.reshape(4, 2))
			# Run tesseract OCR on image
			text = pytesseract.image_to_string(img, config=config)
			
			if not text:
				return success, val1, val2
			
			text.replace('\n', ' ')
			text.replace('\r', ' ')
			
			
			numbers = []
			# look for -, a digit, a dot ending with a digit and a percentage sign
			rx = r'[-\d.]+\d%'

			# loop over the results
			for match in re.finditer(rx, text):
				interval = match.group(0).split('-')
				for number in interval:
					if 0 <= float(number.strip('%')) <= 100:
						numbers.append(number)

			#print '&', numbers[0]
			
			#text2 = re.findall(r'[0-9]', text)
			#print '-->>', text2.groups()
			
			val1 = numbers[0]
			success = True
			val2 = numbers[1]
			
			
			cv2.putText(self.img, val1+' '+val2 , (int(self.size[1]/2), int(self.size[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
		except:
			None
		
		return success, val1, val2
	
		
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
		hsvtresh = cv2.inRange(hsv, hsvMin, hsvMax)
	
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
		#cv2.moveWindow('Blue', int(m*self.size[1]), 1)
		#cv2.imshow('Blue', self.blue)
		#
		#cv2.moveWindow('Yellow', 1, int(m*self.size[0]))
		#cv2.imshow('Yellow', self.yellow)
		#
		#cv2.moveWindow('Final', int(m*self.size[1]), int(m*self.size[0]))
		#cv2.imshow('Final', self.final)
	
	#########################################################################
	# destrutor
	#########################################################################
	def __del__(self):
		cv2.destroyAllWindows()
