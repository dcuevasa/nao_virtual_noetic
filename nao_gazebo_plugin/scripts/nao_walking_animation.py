#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class NaoWalkingAnimation:
    def __init__(self):
        rospy.init_node('nao_walking_animation')
        
        # Parámetros configurables - VELOCIDAD AUMENTADA EXTREMA
        self.update_rate = rospy.get_param('~update_rate', 120.0)  # Hz - Aumentado aún más
        self.animation_speed = rospy.get_param('~animation_speed', 10.0)  # Factor de velocidad extremo
        self.movement_threshold = rospy.get_param('~movement_threshold', 0.005)  # Umbral reducido
        
        # Parámetros para desacoplar la animación de la velocidad real
        # Permitir que la animación vaya más rápido que el límite físico de 0.2
        self.max_animation_speed = 0.5  # Máxima velocidad para animación (mayor que el límite de cmd_vel)
        
        # Tiempo de ejecución de movimientos (segundos) - REDUCIDO AL MÍNIMO
        self.movement_time = 0.1  # Tiempo mínimo viable para los controladores
        
        # Variables de estado
        self.is_moving = False
        self.animation_phase = 0.0  # Fase de la animación (0.0 a 2*pi)
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # CORRECCIÓN: Configuración correcta de publishers y mapeo de articulaciones según pub_utils.py
        self.left_arm_pub = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
        self.right_arm_pub = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
        self.left_hand_pub = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=1)
        self.right_hand_pub = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=1)
        self.pelvis_pub = rospy.Publisher('/nao_dcm/Pelvis_controller/command', JointTrajectory, queue_size=1)
        self.left_leg_pub = rospy.Publisher('/nao_dcm/LeftLeg_controller/command', JointTrajectory, queue_size=1)
        self.right_leg_pub = rospy.Publisher('/nao_dcm/RightLeg_controller/command', JointTrajectory, queue_size=1)
        self.left_foot_pub = rospy.Publisher('/nao_dcm/LeftFoot_controller/command', JointTrajectory, queue_size=1)
        self.right_foot_pub = rospy.Publisher('/nao_dcm/RightFoot_controller/command', JointTrajectory, queue_size=1)
        self.head_pub = rospy.Publisher('/nao_dcm/Head_controller/command', JointTrajectory, queue_size=1)
        
        # Mapeo correcto de grupos de articulaciones para cada parte del cuerpo
        self.joint_groups = {
            'left_arm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
            'right_arm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
            'pelvis': ['LHipYawPitch'],
            'left_leg': ['LHipRoll', 'LHipPitch', 'LKneePitch'],
            'right_leg': ['RHipRoll', 'RHipPitch', 'RKneePitch'],
            'left_foot': ['LAnklePitch', 'LAnkleRoll'],  # Nota: en pub_utils.py es LAnklePitchJoint
            'right_foot': ['RAnklePitch', 'RAnkleRoll']  # Nota: en pub_utils.py es RAnklePitchJoint
        }
        
        # Obtener articulaciones disponibles
        rospy.loginfo("Esperando mensaje joint_states...")
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            self.available_joints = joint_state_msg.name
            rospy.loginfo(f"Articulaciones disponibles: {self.available_joints}")
        except rospy.ROSException:
            rospy.logwarn("No se pudo obtener joint_states. Usando lista predefinida.")
            # Lista predefinida de articulaciones del NAO
            self.available_joints = [
                'HeadYaw', 'HeadPitch',
                'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
                'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
                'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
                'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'
            ]
        
        # Articulaciones para la animación de caminata
        self.walking_joints = [
            # Piernas
            'LHipYawPitch',                      # Pelvis
            'LHipRoll', 'LHipPitch', 'LKneePitch',      # Pierna izquierda
            'LAnklePitch', 'LAnkleRoll',         # Pie izquierdo
            'RHipRoll', 'RHipPitch', 'RKneePitch',      # Pierna derecha
            'RAnklePitch', 'RAnkleRoll',         # Pie derecho
            
            # Brazos
            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',  # Brazo izquierdo
            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'   # Brazo derecho
        ]
        
        # Filtrar solo las articulaciones disponibles
        self.used_joints = [joint for joint in self.walking_joints if joint in self.available_joints]
        rospy.loginfo(f"Usando {len(self.used_joints)} articulaciones para la animación")
        
        # Definir posiciones neutrales para cada articulación
        self.neutral_positions = {
            # Piernas (posición de parado)
            'LHipYawPitch': 0.0,
            'LHipRoll': 0.0, 'LHipPitch': 0.0, 'LKneePitch': 0.0,
            'LAnklePitch': 0.0, 'LAnkleRoll': 0.0,
            'RHipRoll': 0.0, 'RHipPitch': 0.0, 'RKneePitch': 0.0,
            'RAnklePitch': 0.0, 'RAnkleRoll': 0.0,
            
            # Brazos (posición de descanso)
            'LShoulderPitch': 1.5, 'RShoulderPitch': 1.5,      # Brazos abajo
            'LShoulderRoll': 0.15, 'RShoulderRoll': -0.15,     # Ligeramente separados del cuerpo
            'LElbowYaw': 0.0, 'RElbowYaw': 0.0,                # Sin rotación
            'LElbowRoll': -0.5, 'RElbowRoll': 0.5,             # Ligeramente doblados
            'LWristYaw': 0.0, 'RWristYaw': 0.0                 # Sin rotación
        }
        
        # Amplitudes de movimiento para cada articulación durante la caminata
        self.amplitudes = {
            # Piernas
            'LHipYawPitch': 0.1,                             # Ligera rotación pélvica
            'LHipRoll': 0.1, 'RHipRoll': 0.1,                # Balanceo lateral de la cadera
            'LHipPitch': 0.4, 'RHipPitch': 0.4,              # Balanceo adelante/atrás de la cadera
            'LKneePitch': 0.6, 'RKneePitch': 0.6,            # Flexión de la rodilla
            'LAnklePitch': 0.3, 'RAnklePitch': 0.3,          # Flexión del tobillo
            'LAnkleRoll': 0.1, 'RAnkleRoll': 0.1,            # Balanceo lateral del tobillo
            
            # Brazos
            'LShoulderPitch': 0.2, 'RShoulderPitch': 0.2,    # Balanceo adelante/atrás del hombro
            'LShoulderRoll': 0.1, 'RShoulderRoll': 0.1,      # Balanceo lateral del hombro
            'LElbowYaw': 0.1, 'RElbowYaw': 0.1,              # Ligera rotación del codo
            'LElbowRoll': 0.2, 'RElbowRoll': 0.2,            # Flexión del codo
            'LWristYaw': 0.1, 'RWristYaw': 0.1               # Ligera rotación de la muñeca
        }
        
        # Desfases para crear el patrón de caminar
        self.phase_offsets = {
            # Piernas (en radianes)
            'LHipYawPitch': 0.0,                                    # Pelvis
            'LHipRoll': math.pi/2, 'RHipRoll': math.pi + math.pi/2,  # Balanceo lateral en contrafase
            'LHipPitch': 0.0, 'RHipPitch': math.pi,                  # Piernas en contrafase
            'LKneePitch': math.pi/4, 'RKneePitch': math.pi + math.pi/4,  # Ligeramente desfasado
            'LAnklePitch': math.pi/2, 'RAnklePitch': math.pi + math.pi/2,
            'LAnkleRoll': 0.0, 'RAnkleRoll': math.pi,
            
            # Brazos (brazos opuestos a las piernas)
            'LShoulderPitch': math.pi, 'RShoulderPitch': 0.0,     # Brazos balancean opuestos a piernas
            'LShoulderRoll': math.pi/2, 'RShoulderRoll': math.pi + math.pi/2,
            'LElbowYaw': 0.0, 'RElbowYaw': math.pi,
            'LElbowRoll': math.pi/4, 'RElbowRoll': math.pi + math.pi/4,
            'LWristYaw': 0.0, 'RWristYaw': math.pi
        }
        
        # Enviar posición inicial neutral
        self.reset_to_neutral_pose()
        
        # Suscriptor para los comandos de velocidad
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Temporizador para actualizar la animación
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_animation)
        
        rospy.loginfo("Animación de caminata para NAO inicializada")
    
    def cmd_vel_callback(self, msg):
        # Calcular velocidad lineal y angular total
        linear_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        angular_speed = abs(msg.angular.z)
        
        # Escalar la velocidad para la animación - permite animaciones más rápidas
        # aunque la velocidad real del robot esté limitada
        animation_linear_speed = min(self.max_animation_speed, linear_speed * 2.5)
        animation_angular_speed = min(self.max_animation_speed, angular_speed * 2.5)
        
        # Actualizar estado de movimiento
        self.is_moving = linear_speed > self.movement_threshold or angular_speed > self.movement_threshold
        self.current_linear_speed = animation_linear_speed  # Usar la velocidad escalada
        self.current_angular_speed = animation_angular_speed  # Usar la velocidad escalada
        
        if self.is_moving:
            rospy.loginfo(f"Movimiento: real={linear_speed:.2f}, anim={animation_linear_speed:.2f}")
    
    def update_animation(self, event):
        if not self.is_moving:
            # Si no está en movimiento, volver a posición neutral (pero no constantemente)
            self.reset_to_neutral_pose()
            return
        
        # Actualizar fase de animación según velocidad - VELOCIDAD EXTREMA
        # Aumentar artificialmente el speed_factor para animaciones más rápidas
        speed_factor = max(self.current_linear_speed, self.current_angular_speed * 0.5)
        
        # Aplicar un escalado no lineal para que pequeños cambios de velocidad 
        # produzcan grandes cambios en la animación
        enhanced_speed = math.pow(speed_factor * 5.0, 1.5)  # Escalado no lineal
        
        phase_increment = (2 * math.pi * enhanced_speed * self.animation_speed) / self.update_rate
        self.animation_phase = (self.animation_phase + phase_increment) % (2 * math.pi)
        
        # Calcular posiciones para todas las articulaciones
        positions = {}
        for joint in self.used_joints:
            if joint in self.amplitudes and joint in self.phase_offsets:
                # Calcular posición basada en función sinusoidal - AMPLITUDES MAYORES
                amplitude = self.amplitudes[joint] * min(2.0, enhanced_speed)  # Amplitud aumentada
                offset = self.phase_offsets[joint]
                neutral = self.neutral_positions.get(joint, 0.0)
                
                positions[joint] = neutral + amplitude * math.sin(self.animation_phase + offset)
        
        # Enviar comandos a cada grupo de articulaciones de manera separada
        self.send_joint_commands(positions)
    
    def send_joint_commands(self, positions):
        # Método basado en pub_utils.py para enviar comandos de forma correcta
        
        # Ajustar el tiempo en función de la velocidad actual
        # A mayor velocidad, menor tiempo para alcanzar la posición
        current_time = max(0.05, min(0.2, 0.2 / (1.0 + self.current_linear_speed)))
        
        # Left Arm command
        if all(joint in positions for joint in self.joint_groups['left_arm']):
            la = JointTrajectory()
            la.joint_names = self.joint_groups['left_arm']
            point = JointTrajectoryPoint()
            point.positions = [positions[joint] for joint in la.joint_names]
            # TIEMPO DINÁMICO para adaptarse a la velocidad
            point.time_from_start = rospy.Duration.from_sec(current_time)
            la.points.append(point)
            self.left_arm_pub.publish(la)
        
        # Right Arm command
        if all(joint in positions for joint in self.joint_groups['right_arm']):
            ra = JointTrajectory()
            ra.joint_names = self.joint_groups['right_arm']
            point = JointTrajectoryPoint()
            point.positions = [positions[joint] for joint in ra.joint_names]
            # TIEMPO REDUCIDO para movimientos más rápidos
            point.time_from_start = rospy.Duration.from_sec(self.movement_time)
            ra.points.append(point)
            self.right_arm_pub.publish(ra)
        
        # Pelvis command
        if 'LHipYawPitch' in positions:
            p = JointTrajectory()
            p.joint_names = ['LHipYawPitch']
            point = JointTrajectoryPoint()
            point.positions = [positions['LHipYawPitch']]
            # TIEMPO REDUCIDO para movimientos más rápidos
            point.time_from_start = rospy.Duration.from_sec(self.movement_time)
            p.points.append(point)
            self.pelvis_pub.publish(p)
        
        # Left Leg command
        if all(joint in positions for joint in self.joint_groups['left_leg']):
            ll = JointTrajectory()
            ll.joint_names = self.joint_groups['left_leg']
            point = JointTrajectoryPoint()
            point.positions = [positions[joint] for joint in ll.joint_names]
            # TIEMPO REDUCIDO para movimientos más rápidos
            point.time_from_start = rospy.Duration.from_sec(self.movement_time)
            ll.points.append(point)
            self.left_leg_pub.publish(ll)
        
        # Left Foot command
        if all(joint in positions for joint in self.joint_groups['left_foot']):
            lf = JointTrajectory()
            lf.joint_names = self.joint_groups['left_foot']
            point = JointTrajectoryPoint()
            point.positions = [positions[joint] for joint in lf.joint_names]
            # TIEMPO REDUCIDO para movimientos más rápidos
            point.time_from_start = rospy.Duration.from_sec(self.movement_time)
            lf.points.append(point)
            self.left_foot_pub.publish(lf)
        
        # Right Leg command
        if all(joint in positions for joint in self.joint_groups['right_leg']):
            rl = JointTrajectory()
            rl.joint_names = self.joint_groups['right_leg']
            point = JointTrajectoryPoint()
            point.positions = [positions[joint] for joint in rl.joint_names]
            # TIEMPO REDUCIDO para movimientos más rápidos
            point.time_from_start = rospy.Duration.from_sec(self.movement_time)
            rl.points.append(point)
            self.right_leg_pub.publish(rl)
        
        # Right Foot command
        if all(joint in positions for joint in self.joint_groups['right_foot']):
            rf = JointTrajectory()
            rf.joint_names = self.joint_groups['right_foot']
            point = JointTrajectoryPoint()
            point.positions = [positions[joint] for joint in rf.joint_names]
            # TIEMPO REDUCIDO para movimientos más rápidos
            point.time_from_start = rospy.Duration.from_sec(self.movement_time)
            rf.points.append(point)
            self.right_foot_pub.publish(rf)
    
    def reset_to_neutral_pose(self):
        # Posiciones neutrales para todas las articulaciones
        neutral_positions = {}
        for joint in self.used_joints:
            if joint in self.neutral_positions:
                neutral_positions[joint] = self.neutral_positions[joint]
        
        # Enviar a todas las articulaciones
        self.send_joint_commands(neutral_positions)

if __name__ == '__main__':
    try:
        walker = NaoWalkingAnimation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass