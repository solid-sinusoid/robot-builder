from ..base.gazebo import GazeboVisitor
from lxml import etree
from ..elements.gazebo import (
    Gazebo,
    GzFts,
    GzMaterial,
    GzPbr,
    GzPbrMetal,
    GzPlugin,
    GzRos,
    GzSensor,
    GzVisual,
)

class GazeboUrdfParser(GazeboVisitor):

    def visit_gazebo(self, config: etree._Element | dict, element: Gazebo):
        if isinstance(config, dict):
            return NotImplemented
        self.visit_and_add_xml("visual", config, GzVisual, element)
        # self.visit_and_add_xml("sensor", config, GzSensor, element)
        self.visit_and_add_xml("plugin", config, GzPlugin, element)

        for s in config.findall("sensor"):
            s_name = s.get("name")
            s_type = s.get("type")
            if s_name and s_type:
                sensor = GzSensor(name=s_name, type=s_type)
                sensor.visit(s, self)
                element.add(sensor)

        pFJ = config.find("preserveFixedJoint")
        if pFJ is not None:
            element.preserveFixedJoint = bool(pFJ.text)

        self_collide = config.find("self_collide")
        if self_collide is not None:
            element.self_collide = bool(self_collide.text)

    def visit_gz_plugin(self, config: etree._Element | dict, element: GzPlugin):
        if isinstance(config, dict):
            return NotImplemented

        element.filename = config.get("filename", None)
        element.name = config.get("name", None)

        for p in config.findall("parameters"):
            element.parameters = p.text

        self.visit_and_add_xml("ros", config, GzRos, element)

    def visit_gz_sensor(self, config: etree._Element | dict, element: GzSensor):
        if isinstance(config, dict):
            return NotImplemented

        always_on = config.find("always_on")
        if always_on is not None:
            element.always_on = bool(always_on.text)

        update_rate = config.find("update_rate")
        if update_rate is not None:
            if update_rate.text is not None:
                element.update_rate = int(update_rate.text)

        visualize = config.find("visualize")
        if visualize is not None:
            element.visualize = bool(visualize.text)

        topic = config.find("topic")
        if topic is not None:
            element.topic = topic.text

        self.visit_and_add_xml("force_torque", config, GzFts, element)

    def visit_gz_fts(self, config: etree._Element | dict, element: GzFts):
        if isinstance(config, dict):
            return NotImplemented

        frame = config.find("frame")
        if frame is not None:
            element.frame = frame.text

        measure_direction = config.find("measure_direction")
        if measure_direction is not None:
            element.measure_direction = measure_direction.text

    def visit_gz_visual(self, config: etree._Element | dict, element: GzVisual):
        if isinstance(config, dict):
            return NotImplemented

        self.visit_and_add_xml("material", config, GzMaterial, element)

    def visit_gz_material(self, config: etree._Element | dict, element: GzMaterial):
        if isinstance(config, dict):
            return NotImplemented

        diffuse = config.findtext("diffuse")
        if diffuse is not None:
            element.diffuse = [float(i) for i in diffuse.split(" ")]

        specular = config.findtext("specular")
        if specular is not None:
            element.specular = [float(i) for i in specular.split(" ")]

        ambient = config.findtext("ambient")
        if ambient is not None:
            element.ambient = [float(i) for i in ambient.split(" ")]

        emissive = config.findtext("emissive")
        if emissive is not None:
            element.emissive = [float(i) for i in emissive.split(" ")]

        lighting = config.findtext("lighting")
        if lighting is not None:
            element.lighting = bool(lighting)

        self.visit_and_add_xml("pbr", config, GzPbr, element)

    def visit_gz_pbr(self, config: etree._Element | dict, element: GzPbr):
        if isinstance(config, dict):
            return NotImplemented

        self.visit_and_add_xml("metal", config, GzPbrMetal, element)

    def visit_gz_pbr_metal(self, config: etree._Element | dict, element: GzPbrMetal):
        if isinstance(config, dict):
            return NotImplemented

        albedo = config.findtext("albedo_map")
        if albedo is not None:
            element.albedo_map = albedo

        normal = config.findtext("normal_map")
        if normal is not None:
            element.normal_map = normal

        ambient_occlusion = config.findtext("ambient_occlusion_map")
        if ambient_occlusion is not None:
            element.ambient_occlusion_map = ambient_occlusion

        roughness = config.findtext("roughness_map")
        if roughness is not None:
            element.roughness_map = roughness

    def visit_gz_ros(self, config: etree._Element | dict, element: GzRos):
        if isinstance(config, dict):
            return NotImplemented

        namespace = config.findtext("namespace")
        if namespace is not None:
            element.namespace = namespace
