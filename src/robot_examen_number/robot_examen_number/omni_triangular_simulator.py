#!/usr/bin/env python3
"""
omni_triangular_simulator.py
 
Simulador de odometría para robot triangular con 3 ruedas omnidireccionales.
Basado en la estructura de diff_drive_simulator.py pero con cinemática omni.
 
Geometría del robot:
    - Triángulo equilátero, circunradio L = 0.225 m
    - Rueda 0 (frente):      ángulo  90° → β0 = π/2
    - Rueda 1 (atrás-izq):   ángulo 210° → β1 = 7π/6
    - Rueda 2 (atrás-der):   ángulo 330° → β2 = 11π/6
 
Cinemática inversa (body → wheels):
    Cada rueda omni solo puede empujar en la dirección PERPENDICULAR a su eje.
    La componente paralela al eje se desliza libremente (eso es lo omni).
 
    ω_i = (1/R) * [ -sin(βi)·vx  +  cos(βi)·vy  +  L·ω_z ]
 
    donde:
        βi   = ángulo del vértice i 
        vx   = velocidad lineal en X del robot (frame robot)
        vy   = velocidad lineal en Y del robot (frame robot)
        ω_z  = velocidad angular del robot
        R    = radio de la rueda
        L    = circunradio (distancia centro → rueda)
 
"""
 
import rclpy
from rclpy.node import Node
import math
 
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
 
 
# ── Utilidad: quaternion desde yaw ────────────────────────────────────────────
def quaternion_from_yaw(yaw: float) -> tuple:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)
 
 
# ── Ángulos de los vértices del triángulo equilátero ─────────────────────────
#    β0 = 90°, β1 = 210°, β2 = 330°
BETA = [math.pi / 2,
        7 * math.pi / 6,
        11 * math.pi / 6]
 
 
class OmniTriangularSimulator(Node):
 
    def __init__(self):
        super().__init__('omni_triangular_simulator')
 
        # ── Parámetros del robot ──────────────────────────────────────────
        self.declare_parameter('wheel_radius',   0.00)    # radio de cada rueda (m)
        self.declare_parameter('robot_radius',   0.00)   # circunradio triángulo (m)
        self.declare_parameter('update_rate',    50.0)    # ciclos por segundo (Hz)
        self.declare_parameter('max_wheel_vel',  5.0)     # velocidad angular máx rueda (rad/s)
 
        self.R    = self.get_parameter('wheel_radius').value
        self.L    = self.get_parameter('robot_radius').value
        self.rate = self.get_parameter('update_rate').value
        self.wmax = self.get_parameter('max_wheel_vel').value
 
        # ── Estado del robot (odometría) ──────────────────────────────────
        self.x  = 0.0   # posición X en el mundo (m)
        self.y  = 0.0   # posición Y en el mundo (m)
        self.th = 0.0   # orientación yaw (rad)
 
        # ── Velocidades angulares de cada rueda (rad/s) ───────────────────
        #    ω0 = rueda frente, ω1 = atrás-izq, ω2 = atrás-der
        self.omega = [0.0, 0.0, 0.0]
 
        # ── Ángulos acumulados de las ruedas para JointState (RViz) ──────
        self.angle = [0.0, 0.0, 0.0]
 
        # ── Matriz J de cinemática inversa (3×3) ─────────────────────────
        #    Fila i: [ -sin(βi), cos(βi), L ]
        #    Para triángulo equilátero J⁻¹ = (##/##) · Jᵀ  (simplificación)
        self._build_jacobian()
 
        # ── Publicadores ──────────────────────────────────────────────────
        self.pub_odom   = self.create_publisher(Odometry,          '/odom',         10)
        self.pub_pose   = self.create_publisher(Pose2D,            '/robot_pose',   10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.pub_joints = self.create_publisher(JointState,        '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
 
        # ── Suscriptores ──────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd_vel, 10)
 
        self.create_subscription(
            Float32MultiArray, '/wheel_velocities', self._cb_wheel_vel, 10)
 
        # ── Timer principal ───────────────────────────────────────────────
        dt = 1.0 / self.rate
        self.create_timer(dt, self._update)
        self._last_time = self.get_clock().now()
 
        self.get_logger().info(
            f'OmniTriangularSimulator iniciado — '
            f'R={self.R}m  L={self.L}m  rate={self.rate}Hz'
        )
 
    # ── Construcción de la Jacobiana ──────────────────────────────────────────
    def _build_jacobian(self):
        """
        J (3×3): convierte velocidades del cuerpo [vx, vy, ωz] → ωi_rueda
        J_inv  : convierte ωi_rueda → velocidades del cuerpo
 
        COMPLETAR:
            - Cada fila i de J tiene la forma: [ ######, ######, ###### ]
            - J_inv se obtiene a partir de la propiedad del triángulo equilátero:
              J_inv = (##/##) * Jᵀ
        """
        self.J = []
        for b in BETA:
            # COMPLETAR: fila i → [ ######, ######, ###### ]
            self.J.append([######, ######, ######])
 
        # COMPLETAR: calcular J_inv usando la simplificación del triángulo equilátero
        self.J_inv = []
        for col in range(3):
            row = []
            for i in range(3):
                row.append(###### * self.J[i][col])   # ¿qué factor va aquí?
            self.J_inv.append(row)
 
    # ── Callbacks ─────────────────────────────────────────────────────────────
 
    def _cb_cmd_vel(self, msg: Twist):
        """
        Recibe Twist (vx, vy, ω_z) en el frame del robot.
        Aplica CINEMÁTICA INVERSA → ω de cada rueda.
 
        ω_i = (1/R) · [-sin(βi)·vx + cos(βi)·vy + L·ω_z]
        """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
 
        for i in range(3):
            w_i = (self.J[i][0] * vx +
                   self.J[i][1] * vy +
                   self.J[i][2] * wz) / self.R
            self.omega[i] = max(-self.wmax, min(self.wmax, w_i))
 
    def _cb_wheel_vel(self, msg: Float32MultiArray):
        """
        Recibe [ω0, ω1, ω2] en rad/s directamente.
        Control de bajo nivel sin cinemática inversa.
        """
        for i in range(min(3, len(msg.data))):
            self.omega[i] = max(-self.wmax, min(self.wmax, float(msg.data[i])))
 
    # ── Ciclo principal ────────────────────────────────────────────────────────
 
    def _update(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
 
        if dt <= 0.0:
            return
 
        # ── CINEMÁTICA DIRECTA: wheels → body  (ξ vs ω) ──────────────────
        #    Velocidades lineales de cada rueda (m/s)
        v_wheels = [self.omega[i] * self.R for i in range(3)]
 
        #    COMPLETAR: [vx, vy, ωz] = J_inv · v_wheels
        vx = ######
        vy = ######
        wz = ######
 
        # ── INTEGRACIÓN EULER: η̇ vs ξ ──────────────────────────────────
        #    COMPLETAR: rotar velocidades del frame robot al frame mundo
        cos_th = math.cos(self.th)
        sin_th = math.sin(self.th)
 
        vx_world = ######
        vy_world = ######
 
        # ── INTEGRACIÓN: determinar η ──────────────────────────────────
        #    COMPLETAR: integrar posición y orientación con método Euler
        self.th += ######
        self.th  = ######          # Normalizar a [-π, π]
        self.x  += ######
        self.y  += ######
 
        # ── Ángulos acumulados de las ruedas (para RViz) ──────────────────
        for i in range(3):
            self.angle[i] += self.omega[i] * dt
 
        # ── Publicar ──────────────────────────────────────────────────────
        self._publish_odom(now, vx, vy, wz)
        self._publish_pose()
        self._publish_wheel_speeds()
        self._publish_joint_states(now)
        self._broadcast_tf(now)
 
    # ── Publicadores ──────────────────────────────────────────────────────────
 
    def _publish_odom(self, now, vx: float, vy: float, wz: float):
        """
        COMPLETAR: Publicador para mensaje tipo Odometry.
        Debe incluir:
          - header con stamp y frame_id = '######'
          - child_frame_id  = '######'
          - pose: posición (x, y, z) y orientación (quaternion)
          - twist: velocidades lineales (vx, vy) y angular (wz)
        """
        qx, qy, qz, qw = quaternion_from_yaw(self.th)
        msg = Odometry()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = '######'
        msg.child_frame_id  = '######'
 
        # POSE — dónde está el robot
        msg.pose.pose.position.x    = ######
        msg.pose.pose.position.y    = ######
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = ######
        msg.pose.pose.orientation.w = ######
 
        # TWIST — velocidades en el frame del robot
        msg.twist.twist.linear.x  = ######
        msg.twist.twist.linear.y  = ######
        msg.twist.twist.angular.z = ######
 
        self.pub_odom.publish(msg)
 
    def _publish_pose(self):
        """Pose simplificada 2D del robot."""
        msg = Pose2D()
        msg.x     = self.x
        msg.y     = self.y
        msg.theta = self.th
        self.pub_pose.publish(msg)
 
    def _publish_wheel_speeds(self):
        """Velocidades angulares de las 3 ruedas [ω0, ω1, ω2] en rad/s."""
        msg = Float32MultiArray()
        msg.data = [float(self.omega[i]) for i in range(3)]
        self.pub_wheels.publish(msg)
 
    def _publish_joint_states(self, now):
        """
        COMPLETAR: Publicador para mensaje tipo JointState.
        Debe incluir:
          - header con stamp
          - name: lista con los nombres de los 3 joints (deben coincidir con el URDF)
          - position: ángulo acumulado de cada rueda (rad)
          - velocity: velocidad angular de cada rueda (rad/s)
        """
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name     = ['######', '######', '######']   # deben coincidir con el URDF
        msg.position = ######
        msg.velocity = ######
        self.pub_joints.publish(msg)
 
    def _broadcast_tf(self, now):
        """
        COMPLETAR: Transformación que relaciona base_link con respecto a odom.
        Debe definir:
          - frame_id (padre): '######'
          - child_frame_id  : '######'
          - translation: posición actual del robot (x, y, z=0)
          - rotation: orientación actual como quaternion
        """
        qx, qy, qz, qw = quaternion_from_yaw(self.th)
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = '######'    # frame padre
        t.child_frame_id  = '######'    # frame hijo
        t.transform.translation.x = ######
        t.transform.translation.y = ######
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = ######
        t.transform.rotation.w = ######
        self.tf_broadcaster.sendTransform(t)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = OmniTriangularSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 
