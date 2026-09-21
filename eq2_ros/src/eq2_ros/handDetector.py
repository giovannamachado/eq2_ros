from dataclasses import dataclass
import pathlib

import cv2 as cv
import mediapipe as mp
import numpy as np
from math import dist as pointDist
import os



@dataclass
class handDist:#classe que guarda as informações
    x:float
    y:float
    finger_dists: dict#lista de dedos
    comp_fingers: dict
    @property
    def closed(self):
        count = 0
        for points in self.finger_dists.values():

            if all(points[0]<points[k] for k,v in self.comp_fingers.items()):#se distancia da ponta do dedo for menor, o dedo é considerado como fechado
                count+=1
        return count>=4
    def closedFinger(self,n):
        points = self.finger_dists[n]
        return all(points[0]<points[k] for k,v in self.comp_fingers.items())
        
    def __str__(self):
        #return f"handDist(x:{self.x*100:.1f}%,y:{self.y*100:.1f}%,closed:{self.closed})"
        cs = [f"Finger {k:<2}: [{", ".join(f"{f"{n:.1f}":>5}" for n in v)},{self.closedFinger(k)}]" for k,v in self.finger_dists.items()]
        return f"handDist(x:{self.x*100:.1f}%,y:{self.y*100:.1f}%,closed:{self.closed}){"".join(f"\n\t\t{v}" for v in cs)}"
#GIT/ep2_ros/mediapipe/files/hand_landmarker.task"

class handDetection:
    def __init__(self,#varios valores padrão
                 dead_zone_size= (160,90),#limites da zona morta, pode ser int caso o ela seja quadrada, tuple(int,int) para retangulos
                 max_zone_size= (160,90),#limites da zona maxima, similar ao anterior, usa a distancia para borda ao invez do seu tamanho
                 frame_width = 1900,frame_height = 1900,#resolução desejada (no coumputador testado ele transforma em 720x1280)
                 task_path =  os.path.join(pathlib.Path(__file__).parent.resolve(),"files/hand_landmarker.task"),#caminho para o arquivo tsak do mediapipe
                 confidence={"detection":0.5,"presence":0.5,"traking":0.5},#variaveis de confiança do modelo do mediapipe
                 limit= -100,#Quão fora do quadro o centro da mão deve estar para ser desconsiderado
                 cross_mode = False):#O modo de exibição das zonas da imagem
        self.limit = limit if limit<0 else -limit#limite deve ser negativo
        self._running = False
        self.cross_mode = cross_mode
        self.mzone = (max_zone_size[0]/2,max_zone_size[1]/2) if isinstance(max_zone_size,tuple) else (max_zone_size/2,max_zone_size/2)#sempre usa metade do numero entregue
        self.dzone = (dead_zone_size[0]/2,dead_zone_size[1]/2) if isinstance(dead_zone_size,tuple) else (dead_zone_size/2,dead_zone_size/2)
        self.cap = cv.VideoCapture(0, cv.CAP_DSHOW)#inicio da captura
        # self.cap = cv.VideoCapture(0,cv.CAP_V4L2)
        # self.cap.set(cv.CAP_PROP_FOURCC,cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, frame_height)#tenta configurar a resolução da captura, (geralmente resulta em um valor menor)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, frame_width)
        _, self.frame = self.cap.read()#verifica a resolução da captura
        y,x = self.frame.shape[:2]
        self.rez = (x,y) # resolução da captura
        self.center = (int(x/2),int(y/2)) # centro da captura
        self.hand_center = (-x,-y) #centro da mão
        self.duos = [(0,1),(0,5),(0,17),(5,9),(9,13),(13,17)]#duplas de pontos para desenhar linhas em um metodo
        self.duos +=[(v+i-1,v+i) for v in set([vl[1] for vl in self.duos]) for i in range(1,4) if v!=0]+[(2,5)]
        options = mp.tasks.vision.HandLandmarkerOptions(#opçoes do detector
            base_options=mp.tasks.BaseOptions(model_asset_path=task_path),
            min_hand_presence_confidence = confidence["presence"],
            min_hand_detection_confidence= confidence["detection"],  
            min_tracking_confidence= confidence["traking"], 
            running_mode=mp.tasks.vision.RunningMode.IMAGE, 
            num_hands=1, )

        self.detector = mp.tasks.vision.HandLandmarker.create_from_options(options)#detector

    # def _hand_gesture(self):#Reconhece possiveis gestos
    #     pass

    def _hand_dists(self):#detecta se a mão aparenta estar fechada
        point = self.hand_points[0]# ponto base
        fingers = {}# lista de dedos
        for i in [4*j for j in range(1,6)]:
            dists = []
            dists.append(pointDist(self.hand_points[i],point))#distancia da ponta do dedo com o ponto base
            dists.append(pointDist(self.hand_points[i-1],point))#distancia do segundo ponto do dedo com o ponto base
            dists.append(pointDist(self.hand_points[i-2],point))
            dists.append(pointDist(self.hand_points[i-3],point))
            fingers[i] = dists
            
        return fingers
    
    def _dist_center(self,p,i):#calcula onde o ponto central da mão esta, em relação ao limite das duas zonas
        side = self.rez[i]
        
        if p>=self.limit and p<=side-self.limit:#verifica se esta dentro dos limites aceitos
            d_zone = self.dzone[i]
            dist = side/2-d_zone#distancia para zona morta
            if p>(d_zone+side/2): #caso esteja depois da zona morta
                return min((p-(d_zone+side/2))/(dist-self.mzone[i]),1.0)#distancia entre mão e zona morta/ distancia entra as duas zonas
            elif p<(dist): #caso antes da zona
                return max((p-(dist))/(dist-self.mzone[i]),-1.0)#distancia entre mão e zona morta/ distancia entra as duas zonas
        return 0

    @property
    def is_running(self):# Verifica se esta rodando
        return self._running

    def stop_running(self):
        self._running = False

    @property
    def hand_dist(self):# cria um objeto handDist
        px,py = (float(self.hand_center[0]),float(self.hand_center[1]))
        x = self._dist_center(px,0)
        y = -self._dist_center(py,1)
        fing = self._hand_dists()
        return handDist(x,y,finger_dists=fing,comp_fingers={2:1.0})#(x,y,closed)

    # def updatedefhandDist(self):
    #     px,py = (float(self.hand_center[0]),float(self.hand_center[1]))
    #     x = self._dist_center(px,0)
    #     y = -self._dist_center(py,1)
    #     fing = self._hand_dists()
    #     self.__hand_dist = handDist(x,y,finger_dists=fing,comp_fingers={2:1.0})
    

    def __doubleLine(self,p1,p2,color,color2):#desenha duas linhas uma em cima da outra
        cv.line(self.frame,p1,p2,color=color,thickness=2)
        cv.line(self.frame,p1,p2,color=color2,thickness=1)

    def __doublePoint(self,p1,color,color2,size = 2):#desenha dois pontos um em cima da outro
        cv.circle(self.frame,p1,size+1,color=color,thickness=2)
        cv.circle(self.frame,p1,size,color=color2,thickness=1)

    def __drawnZones(self):# desenha a zona morta e zona maxima
        zx = int(self.dzone[0])
        zy = int(self.dzone[1])
        mx = int(self.mzone[0])
        my = int(self.mzone[1])
        (cx,cy) = self.center#centro da tela
        rx,ry = self.rez
        if self.cross_mode:#usa retangulos para desenhar linhas que começão e terminam fora da imagem
            duos = [((cx+zx,-10),(cx-zx,ry+4),(0,0,0)),((-10,zy+cy),(rx+10,cy-zy),(0,0,0)),#zona morta
                    ((mx,-10),(rx-mx,ry+4),(255,255,255)),((-10,my),(rx+10,ry-my),(255,255,255))]#zona maxima
        else: 
            duos = [((cx+zx,zy+cy),(cx-zx,cy-zy),(0,0,0)),#zona morta
                    ((mx,my),(rx-mx,ry-my),(255,255,255))]#zona maxima
        self.__doublePoint((cx,cy),(255,255,255),(0,0,0))#ponto central da tela
        for p1,p2,color in duos:#Cria os retangulos
            cv.rectangle(self.frame,pt1=p1,pt2=p2,color=color,thickness=3)

    def __drawHandAndBox(self,box,cat,id): 
        (x,y) =(int(self.hand_center[0]),int(self.hand_center[1]))#poisição do centro da mão em inteiros
        #linha para do centro da imagem para o centro da mão
        (cx,cy) = self.center
        self.__doubleLine((cx,cy),(x,cy),(127,127,0),(127,0,255))
        self.__doubleLine((x,cy),(x,y),(127,127,0),(127,0,255))
        #desenha um retangulo ao redor dos pontos da mão
        dist = self.hand_dist
        cv.drawContours(self.frame,[box],contourIdx=0,color=(255,0,0),thickness=2)
        self.__doublePoint((x,y),(0,0,255),(0,255,0),size=3)#ponto central do retangulo
        cv.putText(self.frame,f"{cat}: {dist}".replace("\t","    "),(x,y),cv.FONT_HERSHEY_PLAIN,1,(0,255,255))#Categoria(Lado) e status da mão
        #desenha linhas entre os dedos da mão
        for a,b in self.duos:#usa as duplas de indexes dos pontos para desenhar as linhas
            self.frame = cv.line(self.frame,self.hand_points[a],self.hand_points[b],color=(0,255,0))
        for p in self.hand_points:#desenha cada ponto da mão e numera eles
            self.frame = cv.putText(self.frame,f"{self.hand_points.index(p)}",p,cv.FONT_HERSHEY_PLAIN,1,(255,255,0))
            self.frame = cv.circle(self.frame,p,2,color=(255,0,255),thickness=-1)
        print(f"{id}\n\tSide:{cat}\n\tDist:{dist}")# print para as informações das mãos
        
    def main(self):
        
        handSwitch = {0:'Left',1:'Right'}#corrige o lado das mãos
        x,y = self.rez
        cv.namedWindow('Webcam', cv.WINDOW_KEEPRATIO)
        self._running = True
        while self._running:
            ret, frame = self.cap.read()
            self.frame = cv.flip(frame,1)
            if not ret: continue
            frame_RGB = mp.Image(mp.ImageFormat.SRGB,cv.cvtColor(self.frame,cv.COLOR_BGR2RGB))
            detected = self.detector.detect(frame_RGB)#resultado da detecção
            size = len(detected.hand_landmarks)
            self.__drawnZones()#desenha a zona morta
            if size>0:#ignora se nenuma mão for detectada
                self.hand_points = [(int(l.x*x),int(l.y*y)) for l in detected.hand_landmarks[0]]#pontos da mão

                r = cv.minAreaRect(np.array([self.hand_points]))# pega os pontos da  e centro da mão
                self.hand_center = r[0]#centro da mão
                box =  cv.boxPoints(r) #pontos da caixa
                self.__drawHandAndBox(box.astype(np.int64),handSwitch[detected.handedness[0][0].index],"Main Hand Stats:")
            cv.imshow('Webcam', self.frame)#mostra a imagem capturada com as alterações feitas
            if cv.waitKey(1) & 0xFF == ord('q'): break

        self.cap.release()
        cv.destroyAllWindows()
        self._running = False


if __name__ == "__main__":
    d = {"detection":0.4,"presence":0.4,"traking":0.6}
    det = handDetection(cross_mode=True)
    det.main()

else:
    print(f"Other HandD: {__name__}")