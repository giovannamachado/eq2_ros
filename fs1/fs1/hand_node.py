"""
Operator webcam node: detects one hand via MediaPipe, publishes its
position/open-closed state as ``/hand_status``, and republishes the
annotated frame as ``/camera/operator/image_raw`` for the front-end.
"""

import json
from dataclasses import dataclass
import pathlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2 as cv
import mediapipe as mp
import numpy as np
from math import dist as pointDist
import os

print(cv)

def fun_defaut_path(base = "files/hand_landmarker.task"):
    '''Encontra o arquivo hand_landmarker caso um caminho não seja disponibilizado\n
    Find ``base`` by walking up from this file until it exists (dev or install layout).'''
    p1 = pathlib.Path(__file__).parent.resolve()
    i = 0
    extra = "src/eq2_ros/fs1"
    p2 = os.path.join(p1.parents[i],base)
    p3 =  os.path.join(p1.parents[i],extra,base)
    while not os.path.exists(p2) and not os.path.exists(p3):
        i+=1 
        p2 = os.path.join(p1.parents[i],base)
        p3 = os.path.join(p1.parents[i],extra,base)
    return p2 if os.path.exists(p2) else p3
def _parse_camera_index(value):
    """
    Turn the ``camera_index`` parameter into what ``cv2.VideoCapture`` wants.

    Accepts a plain number (``0``, ``"2"``) or a stable device path (e.g.
    ``/dev/v4l/by-id/usb-Logitech_BRIO-video-index0``, which keeps working
    after the camera is unplugged and plugged into a different USB port,
    unlike ``/dev/videoN``'s number).
    """
    text = str(value)
    return int(text) if text.lstrip('-').isdigit() else text


@dataclass
class handDist:
    """Hand position (x, y, relative to the dead/max zones) and per-finger distances."""

    x:float #classe que guarda as informações
    y:float
    finger_dists: dict#lista de dedos
    comp_fingers: dict

    @property
    def closed(self):
        """Dertermine that is closed if at least 4 fingers are counted as closed."""
        count = 0
        if all(p <= 0 for points in self.finger_dists.values() for p in points): return False
        for points in self.finger_dists.values():
            #se distancia da ponta do dedo for menor, o dedo é considerado como fechado
            if all(points[0]<points[k] for k in self.comp_fingers): count+=1
        return count>=4

    def closedFinger(self,n):
        """True if finger ``n`` is counted as closed."""
        points = self.finger_dists[n]
        if all(p <= 0 for p in points):return False
        return all(points[0]<points[k] for k in self.comp_fingers)
    @property
    def toStr2(self):
        """Multi-line human-readable summary (position + per-finger closed state)."""
        return f"handDist(x:{self.x*100:.1f}%,y:{self.y*100:.1f}%,closed:{self.closed})"+"".join(f"\n\t\t{v}" for v in 
                                                                                                 [f"Finger {k:<2}: [{self.closedFinger(k)}]" 
                                                                                                  for k in self.finger_dists])
    @property
    def jDict(self):
        """Serialize to the JSON payload published on ``/hand_status``."""
        d = {}#"closed_fingers": [i for i in self.finger_dists if self.closedFinger(i)]}
        d["closed"] = self.closed
        if self.x>0:  d["direita"] =  self.x
        elif self.x<0:d["esquerda"]= -self.x
        if self.y>0:  d["cima"]    =  self.y
        elif self.y<0:d["baixo"]   = -self.y
        return json.dumps(d)
    
    def __str__(self):
        """Multi-line string with exact per-finger distances (debug use)."""
        cs = []
        for k,v in self.finger_dists.items():
             s = ", ".join(f"{round(n,1):>5}" for n in v)
             cs.append(f"Finger {k:<2}: [{s},{self.closedFinger(k)}]")
        return f"handDist(x:{self.x*100:.1f}%,y:{self.y*100:.1f}%,closed:{self.closed})"+"".join(f"\n\t\t{v}" for v in cs)

class HandNode(Node):
    """Detects a hand in the operator webcam and publishes its position/state."""

    def __init__(self,dead_zone_size= (120,90),#limites da zona morta, pode ser int caso o ela seja quadrada, tuple(int,int) para retangulos
                max_zone_size= (120,90),#limites da zona maxima, similar ao anterior, usa a distancia para borda ao invez do seu tamanho

                frame_width = 720,frame_height = 400,#resolução desejada (no coumputador testado ele tem maxima 720x1280)

                task_path =  None,#caminho para o arquivo tsak do mediapipe
                confidence={"detection":0.5,"presence":0.5,"traking":0.5},#variaveis de confiança do modelo do mediapipe
                limit= -100,#Quão fora do quadro o centro da mão deve estar para ser desconsiderado
                frame_jump = 3,

                open_window = True,
                cross_mode = False,#O modo de exibição das zonas da imagem
                print_mode = False,
                test_mode:dict|bool = False):#aciona o run no final do init para testes
        """Load the MediaPipe hand landmarker and create the ROS publishers/subscriptions."""
        super().__init__("hand_node")
        # Índice da câmera do operador (webcam do notebook). No laboratório é
        # um dispositivo diferente do da câmera do efetuador (usada pelo
        # vision_node); ajuste via --ros-args -p camera_index:=<n> em vez de
        # editar o código. Pode ser um número (0, 1, 2...) ou um caminho
        # estável, tipo /dev/v4l/by-id/usb-<algo>-video-index0 (não muda ao
        # trocar de porta USB, diferente do número, que muda).
        self.declare_parameter('camera_index', '0')
        self.camera_index = _parse_camera_index(
            self.get_parameter('camera_index').value)
        
        if task_path == None: task_path = fun_defaut_path()
        self.limit = limit if limit<0 else -limit#limite deve ser negativo
        self.frame_jump = False if not frame_height or frame_jump<=1 else frame_jump-1
        self._running = False
        self.print_mode = print_mode
        self.cross_mode = cross_mode

        self.mzone = (max_zone_size[0]/2,max_zone_size[1]/2) if isinstance(max_zone_size,tuple) else (max_zone_size/2,max_zone_size/2)#sempre usa metade do numero entregue
        self.dzone = (dead_zone_size[0]/2,dead_zone_size[1]/2) if isinstance(dead_zone_size,tuple) else (dead_zone_size/2,dead_zone_size/2)
        self.rez = (frame_width,frame_height) # usado na criação da captura, é subistituido pela resolução resultante caso ela seja diferente
        self.center = (limit,limit) # temporario
        self.hand_center = (limit-1,limit-1) #temporario
        self.hand_points = [(0,0) for _ in range(21)]

        self.duos = [(0,1),(0,5),(0,17),(5,9),(9,13),(13,17)]#duplas de pontos para desenhar linhas em um metodo
        self.duos +=[(v+i-1,v+i) for v in set([vl[1] for vl in self.duos]) for i in range(1,4) if v!=0]+[(2,5)]
        #opçoes do detector
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=task_path), min_hand_presence_confidence = confidence["presence"],
            min_hand_detection_confidence= confidence["detection"],  
            min_tracking_confidence= confidence["traking"], running_mode=mp.tasks.vision.RunningMode.IMAGE, num_hands=1, )
        self.detector = mp.tasks.vision.HandLandmarker.create_from_options(options) #detector

        self.send_hand = self.create_publisher(String,"/hand_status",10)
        self.stater_stopper = self.create_subscription(String,"/switchHandDetection",self.switch_running,10)
        #publica a imagem da câmera do operador para o front-end (ver frontend.launch.py)
        self.image_pub = self.create_publisher(Image,"/camera/operator/image_raw",10)
        self.get_logger().info("Hand node started.")
        self.open_window = open_window
        if test_mode and isinstance(test_mode,dict):
            self.run(limit=test_mode["limit"])

    def _publish_frame(self):#monta a mensagem Image sem cv_bridge (evita depender dele aqui)
        """Publish ``self.frame`` as a raw ``sensor_msgs/Image`` (bgr8), no cv_bridge needed."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = self.frame.shape[:2]
        msg.encoding = "bgr8"
        msg.step = self.frame.shape[1]*3
        msg.data = self.frame.tobytes()
        self.image_pub.publish(msg)
        
    
    def _dist_center(self,p,i):
        """Return the hand's offset (-1..1) past the dead zone along axis ``i``, or 0 inside it."""
        side = self.rez[i]
        
        if p>=self.limit and p<=side-self.limit:#verifica se esta dentro dos limites aceitos
            d_zone = self.dzone[i]
            dist = side/2-d_zone#distancia para zona morta
            if p>(d_zone+side/2): #caso esteja depois da zona morta
                return min((p-(d_zone+side/2))/(dist-self.mzone[i]),1.0)#distancia entre mão e zona morta/ distancia entra as duas zonas
            elif p<(dist): #caso antes da zona
                return max((p-(dist))/(dist-self.mzone[i]),-1.0)#distancia entre mão e zona morta/ distancia entra as duas zonas
        return 0

    def switch_running(self,msg):
        """Handle ``/switchHandDetection``: apply any tuning params and toggle detection on/off."""
        param = json.loads(msg.data)
        if "frame_jump" in param: 
            self.frame_jump = param["frame_jump"] if param["frame_jump"] and param["frame_jump"]>1 else False
        if "dzone" in param:
            dead_zone_size = param["dzone"]
            if isinstance(dead_zone_size,list) or isinstance(dead_zone_size,tuple): self.dzone = (dead_zone_size[0]/2,dead_zone_size[1]/2)
            else: self.dzone = (dead_zone_size/2,dead_zone_size/2)
        if "mzone" in param:
            m_zone_size = param["mzone"]
            if isinstance(m_zone_size,list) or isinstance(m_zone_size,tuple): self.dzone = (m_zone_size[0]/2,m_zone_size[1]/2)
            elif isinstance(m_zone_size,int): self.mzone = (m_zone_size/2,m_zone_size/2)
        if "rez" in param:
            rez = param["rez"]
            if isinstance(rez,tuple): self.rez = rez
            elif isinstance(rez,list):self.rez = (rez[0],rez[1])   
            elif isinstance(rez,int): self.rez = (rez,rez)
        self._running = not self._running
        if self._running:
            self.run(limit=param.get("limit",False))

    @property
    def hand_dist(self):# cria um objeto handDist
        """Build a ``handDist`` from the current ``hand_center``/``hand_points``."""
        #distancia dos pontos do dedo
        point = self.hand_points[0] #ponto base
        fing = {} #lista de dedos
        for i in [4*j for j in range(1,6)]:
            dists = []
            dists.append(pointDist(self.hand_points[i],point))   #distancia da ponta do dedo com o ponto base
            dists.append(pointDist(self.hand_points[i-1],point))
            dists.append(pointDist(self.hand_points[i-2],point))
            dists.append(pointDist(self.hand_points[i-3],point))
            fing[i] = dists   

        return handDist(round( self._dist_center(float(self.hand_center[0]),0),5),
                        round(-self._dist_center(float(self.hand_center[1]),1),5),
                        finger_dists=fing, comp_fingers={2:1.0})

    def _doubleLine(self,p1,p2,color,color2):#desenha duas linhas uma em cima da outra
        """Draw a thick ``color`` line with a thin ``color2`` line on top (p1 to p2)."""
        cv.line(self.frame,p1,p2,color=color,thickness=2)
        cv.line(self.frame,p1,p2,color=color2,thickness=1)

    def _doublePoint(self,p1,color,color2,size = 2):#desenha dois pontos um em cima da outro
        """Draw a ``color`` dot with a smaller ``color2`` dot on top, at ``p1``."""
        cv.circle(self.frame,p1,size+1,color=color,thickness=2)
        cv.circle(self.frame,p1,size,color=color2,thickness=1)

    def sendSTOP(self):
        """Publish ``{"STOP": true}`` on ``/hand_status`` (hand left the frame)."""
        msg = String()
        msg.data = json.dumps({"STOP":True})
        self.send_hand.publish(msg)
        
    def _drawnInfo(self,box,id): # publica o estado da mão
        """Draw the debug overlay (zones, hand skeleton, status text) and publish ``/hand_status``."""
        dist = self.hand_dist
        (cx,cy) = self.center
        (x,y) =(int(self.hand_center[0]),int(self.hand_center[1]))

        self._doublePoint((cx,cy),(255,255,255),(0,0,0))#ponto central da tela
        for p1,p2,color in self.areas:#Cria os retangulos
            cv.rectangle(self.frame,pt1=p1,pt2=p2,color=color,thickness=3)

        #linha para do centro da imagem para o centro da mão
        self._doubleLine((cx,cy),(x,cy),(127,127,0),(127,0,255))
        self._doubleLine((x,cy),(x,y),(127,127,0),(127,0,255))
        #desenha um retangulo ao redor dos pontos da mão
        #cv.drawContours(self.frame,[box],contourIdx=0,color=(255,0,0),thickness=2)
        self._doublePoint((x,y),(0,0,255),(0,255,0),size=3)#ponto central do retangulo
        cv.putText(self.frame,f"{dist.toStr2}".replace("\t","    "),(10,20),cv.FONT_HERSHEY_PLAIN,1,(255,0,0))#Categoria(Lado) e status da mão
        #desenha linhas entre os dedos da mão
        for a,b in self.duos: cv.line(self.frame,self.hand_points[a],self.hand_points[b],color=(0,255,0))
        for p in self.hand_points:#desenha cada ponto da mão e numera eles
            #cv.putText(self.frame,f"{self.hand_points.index(p)}",p,cv.FONT_HERSHEY_PLAIN,1,(255,255,0))
            cv.circle(self.frame,p,2,color=(255,0,255),thickness=-1)
        msg = String()
        msg.data = dist.jDict
        self.send_hand.publish(msg)
        if self.print_mode: print(f"{id}\n\tDist:{dist}")# print para as informações das mãos
        
    def run(self,limit = False):
        """Open the camera and loop: detect the hand each frame and publish its state."""
        self.get_logger().info("Running Detector")
        #cap = cv.VideoCapture(0, cv.CAP_DSHOW)
        # cv.CAP_V4L2 explícito: ver vision_node.py (mesma correção -- sem
        # isso, um camera_index em string cai no backend GStreamer e falha
        # com "uridecodebin" ao tentar abrir um device node V4L2 por path).
        cap = cv.VideoCapture(self.camera_index, cv.CAP_V4L2)
        #tenta configurar a resolução da captura, (geralmente resulta em um valor menor)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.rez[1])
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self.rez[0])
        if self.open_window:cv.namedWindow('Webcam', cv.WINDOW_KEEPRATIO)
        #verifica a resolução da captura
        _, self.frame = cap.read()
        y,x = self.frame.shape[:2]
        self.rez = (x,y)
        self.get_logger().info(f"REZ: {self.rez}")
        # centro da captura
        self.center = (int(x/2),int(y/2))
        self._running = True
        hand_detected_in_prev = False
        (cx,cy) = self.center#centro da tela
        
        # desenha a zona morta e zona maxima
        #usa retangulos para desenhar linhas
        if self.cross_mode: 
            rx,ry = self.rez
            self.areas = [((cx+int(self.dzone[0]),-10),
                     (cx-int(self.dzone[0]),ry+4),(0,0,0)),
                    ((-10,int(self.dzone[1])+cy),
                     (rx+10,cy-int(self.dzone[1])),(0,0,0)),#zona morta
                    ((int(self.mzone[0]),-10),
                     (rx-int(self.mzone[0]),ry+4),(255,255,255)),
                    ((-10,int(self.mzone[1])),
                     (rx+10,ry-int(self.mzone[1])),(255,255,255))]#zona maxima
        else: 
            self.areas = [((cx+int(self.dzone[0]),int(self.dzone[1])+cy),
                     (cx-int(self.dzone[0]),cy-int(self.dzone[1])),(0,0,0)),#zona morta
                    ((int(self.mzone[0]),int(self.mzone[1])),
                     (self.rez[0]-int(self.mzone[0]),self.rez[1]-int(self.mzone[1])),(255,255,255))]#zona maxima
        if self.frame_jump: frame_counter = 0
        if limit: counter = 0
        while self._running:
            ret, frame = cap.read()
            if not ret: continue
            if self.frame_jump:
                frame_counter+=1
                if self.frame_jump == frame_counter: frame_counter=0
                else: continue
            self.frame = cv.flip(frame,1)
            frame_RGB = mp.Image(mp.ImageFormat.SRGB,cv.cvtColor(self.frame,cv.COLOR_BGR2RGB))
            detected = self.detector.detect(frame_RGB)#resultado da detecção
            size = len(detected.hand_landmarks)
            #dezenha as areas e ponto central
            self._doublePoint(self.center,(255,255,255),(0,0,0))#ponto central da tela
            for p1,p2,color in self.areas:#Cria os retangulos
                cv.rectangle(self.frame,pt1=p1,pt2=p2,color=color,thickness=3)
            if size>0:
                hand_detected_in_prev = True
                self.hand_points = [(int(l.x*x),int(l.y*y)) for l in detected.hand_landmarks[0]]#pontos da mão
                r = cv.minAreaRect(np.array([self.hand_points]))# pega os pontos da  e centro da mão
                self.hand_center = r[0]#centro da mão
                box =  cv.boxPoints(r) #pontos da caixa
                self._drawnInfo(box.astype(np.int64),"Main Hand Stats:")
            elif hand_detected_in_prev:#avisa no primeiro frame sem mão
                self.sendSTOP()
                hand_detected_in_prev = False
                
            if self.open_window:cv.imshow('Webcam', self.frame)#mostra a imagem capturada com as alterações feitas
            self._publish_frame()#publica o frame (com as zonas/mão desenhadas) para o front-end
            if limit: 
                print(f"limit {counter}")
                counter+=1
                if counter == limit: break
            if (cv.waitKey(1) & 0xFF == ord('q')): break
        cap.release()
        if self.open_window:cv.destroyAllWindows()
        self._running = False
        self.get_logger().info("Detector Closed")


def main(args=None):# pragma: no cover
    """Entry point for the hand-tracking node."""
    #d = {"detection":0.4,"presence":0.4,"traking":0.6}
    rclpy.init(args=args)
    node = HandNode(cross_mode=True,frame_jump=3)
    try: 
        rclpy.spin(node)
        node.switch_running(json.dumps({}))
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__": main()# pragma: no cover
else: print(f"Other HandD: {__name__}")