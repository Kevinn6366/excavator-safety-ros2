from pathlib import Path
import json, math, xml.etree.ElementTree as E
from core import CAMERA_POSITIONS

P = Path(__file__).resolve().parent
W = P / "worlds"
W.mkdir(exist_ok=True)


def sub(p, t, text=None, **a):
    e = E.SubElement(p, t, a)
    if text is not None:
        e.text = str(text)
    return e


def box(link, name, pose, size, color, collision=True):
    for kind in (["visual", "collision"] if collision else ["visual"]):
        v = sub(link, kind, name=name + "_" + kind)
        sub(v, "pose", pose)
        sub(sub(sub(v, "geometry"), "box"), "size", size)
        if kind == "visual":
            m = sub(v, "material")
            sub(m, "ambient", color)
            sub(m, "diffuse", color)


def plugin(p, file, name):
    return sub(p, "plugin", filename=file, name=name)


r = E.Element("sdf", version="1.9")
w = sub(r, "world", name="safety")
ph = sub(w, "physics", name="default", type="ignored")
sub(ph, "max_step_size", 0.002)
sub(ph, "real_time_factor", 1)
for f, n in [
    ("physics", "Physics"),
    ("user-commands", "UserCommands"),
    ("scene-broadcaster", "SceneBroadcaster"),
    ("contact", "Contact"),
]:
    plugin(w, "gz-sim-" + f + "-system", "gz::sim::systems::" + n)
s = plugin(w, "gz-sim-sensors-system", "gz::sim::systems::Sensors")
sub(s, "render_engine", "ogre2")
scene = sub(w, "scene")
sub(scene, "ambient", "0.6 0.6 0.6 1")
sub(scene, "background", "0.35 0.45 0.55 1")
l = sub(w, "light", name="sun", type="directional")
sub(l, "pose", "0 0 15 0 0 0")
sub(l, "diffuse", "0.9 0.9 0.9 1")
sub(l, "direction", "-0.5 0.2 -1")
g = sub(w, "model", name="ground")
sub(g, "static", "true")
box(
    sub(g, "link", name="link"),
    "floor",
    "0 0 -0.1 0 0 0",
    "800 200 0.2",
    "0.35 0.35 0.35 1",
)
# Ground markings provide visible scale across the expanded worksite.
for n in range(-39, 40):
    marker = sub(w, "model", name=f"lane_{n}")
    sub(marker, "static", "true")
    ll = sub(marker, "link", name="link")
    for y in [-5, 5]:
        box(
            ll,
            f"line_{y}",
            f"{n*10} {y} .015 0 0 0",
            "4 .12 .02",
            "0.8 0.8 0.72 1",
            False,
        )
m = sub(w, "model", name="excavator")
sub(m, "pose", "0 0 0 0 0 0")
b = sub(m, "link", name="base_link")
i = sub(b, "inertial")
sub(i, "mass", 4000)
ii = sub(i, "inertia")
for t, v in [("ixx", 2500), ("iyy", 3000), ("izz", 3500)]:
    sub(ii, t, v)
collision = sub(b, "collision", name="body_collision")
sub(collision, "pose", "0 0 0.7 0 0 0")
sub(sub(sub(collision, "geometry"), "box"), "size", "2.6 1.8 1.2")
contact = sub(b, "sensor", name="body_contact", type="contact")
sub(contact, "always_on", "true")
sub(contact, "update_rate", 20)
sub(contact, "topic", "/safety/contacts")
sub(sub(contact, "contact"), "collision", "body_collision")
# Github visual imported by import_model.py; collision remains an explicit conservative box.
visual_file = P / "models" / "github_visuals.xml"
if visual_file.exists():
    for v in E.parse(visual_file).getroot():
        b.append(v)
for side, yaw in [
    ("front", 0),
    ("left", math.pi / 2),
    ("rear", math.pi),
    ("right", -math.pi / 2),
]:
    cx, cy, cz = CAMERA_POSITIONS[side]
    c = sub(b, "sensor", name=side, type="rgbd_camera")
    sub(c, "pose", f"{cx} {cy} {cz} 0 0.65 {yaw}")
    sub(c, "topic", "/safety/" + side)
    sub(c, "update_rate", 10)
    sub(c, "always_on", "true")
    cam = sub(c, "camera")
    sub(cam, "horizontal_fov", 2.2)
    im = sub(cam, "image")
    sub(im, "width", 320)
    sub(im, "height", 240)
    sub(im, "format", "R8G8B8")
    cl = sub(cam, "clip")
    sub(cl, "near", 0.1)
    sub(cl, "far", 15)
vc = plugin(m, "gz-sim-velocity-control-system", "gz::sim::systems::VelocityControl")
sub(vc, "topic", "/safety/cmd_vel")
o = plugin(m, "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher")
sub(o, "odom_publish_frequency", 20)
sub(o, "odom_topic", "/safety/odom")
sub(o, "odom_frame", "world")
sub(o, "robot_base_frame", "base_link")
sub(o, "dimensions", 3)
# Fixed, repeatable random scenario; targets are repositioned by scenario.py.
for n in range(80):
    person = n % 2 == 0
    mm = sub(w, "model", name=f"target_{n}")
    sub(mm, "static", "true")
    sub(
        mm,
        "pose",
        f"{8+(n//2)*9 if n<48 else -350-n*.3} {(-1)**n*(8+(n%4)*2) if n<48 else 80} 0 0 0 0",
    )
    ll = sub(mm, "link", name="link")
    if n < 4:
        mm.find("pose").text = f"{4 if n<2 else -3} {4.5 if n%2==0 else -4.5} 0 0 0 0"
    h = 1.7 if person else 0.8
    box(
        ll,
        "target",
        f"0 0 {h/2} 0 0 0",
        f"0.5 0.5 {h}" if person else "0.8 0.8 0.8",
        "1 0.02 0.02 1" if person else "0.02 0.05 1 1",
    )
E.indent(r)
E.ElementTree(r).write(W / "safety.sdf", encoding="unicode", xml_declaration=True)
bridges = []


def bridge(topic, ros, gz, direction="GZ_TO_ROS"):
    bridges.append(
        dict(
            ros_topic_name=topic,
            gz_topic_name=topic,
            ros_type_name=ros,
            gz_type_name=gz,
            direction=direction,
        )
    )


bridge("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock")
for side in ["front", "left", "rear", "right"]:
    for suffix, ros, gz in [
        ("image", "Image", "Image"),
        ("depth_image", "Image", "Image"),
        ("camera_info", "CameraInfo", "CameraInfo"),
    ]:
        bridge(f"/safety/{side}/{suffix}", "sensor_msgs/msg/" + ros, "gz.msgs." + gz)
bridge("/safety/contacts", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts")
bridges[-1][
    "gz_topic_name"
] = "/world/safety/model/excavator/link/base_link/sensor/body_contact/contact"
bridge("/safety/odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry")
bridge("/safety/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ")
import yaml

(P / "bridge.yaml").write_text(yaml.safe_dump(bridges))
