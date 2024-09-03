from lxml import etree

from ..base.gazebo import GazeboVisitor
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


class GazeboUrdfWriter(GazeboVisitor):
    def __init__(self) -> None:
        super().__init__()

    # INFO: Gazebo part
    def visit_gazebo(self, config: etree._Element | dict, element: Gazebo):
        if isinstance(config, dict):
            return NotImplemented
        attrib = {}
        if element.reference:
            attrib["reference"] = element.reference
        gazebo_xml = etree.SubElement(config, "gazebo", attrib=attrib)

        self_collide_xml = etree.SubElement(gazebo_xml, "self_collide")
        if element.self_collide is not None:
            self_collide_xml.text = str(element.self_collide).lower()

        self._writing(gazebo_xml, element)

    def visit_gz_plugin(self, config: etree._Element | dict, element: GzPlugin):
        if isinstance(config, dict):
            return NotImplemented

        attrib = {}
        if element.name:
            attrib["name"] = element.name
        if element.filename:
            attrib["filename"] = element.filename

        plugin_xml = etree.SubElement(config, "plugin", attrib=attrib)

        element.filename = config.get("filename", None)
        element.name = config.get("name", None)

        self._writing(plugin_xml, element)

    def visit_gz_sensor(self, config: etree._Element | dict, element: GzSensor):
        if isinstance(config, dict):
            return NotImplemented

        attrib = {"name": element.name, "type": element.type}

        gzsensor = etree.SubElement(config, "sensor", attrib=attrib)

        always_on = etree.SubElement(gzsensor, "always_on")
        always_on.text = str(element.always_on)

        update_rate = etree.SubElement(gzsensor, "update_rate")
        update_rate.text = str(element.update_rate)

        visualize = etree.SubElement(gzsensor, "visualize")
        visualize.text = str(element.visualize)

        topic = etree.SubElement(gzsensor, "topic")
        topic.text = element.topic

        self._writing(gzsensor, element)

    def visit_gz_fts(self, config: etree._Element | dict, element: GzFts):
        if isinstance(config, dict):
            return NotImplemented

        ft_sens = etree.SubElement(config, "force_torque")
        frame_xml = etree.SubElement(ft_sens, "frame")
        frame_xml.text = element.frame

        measure_direction_xml = etree.SubElement(ft_sens, "measure_direction")
        measure_direction_xml.text = element.measure_direction

    def visit_gz_visual(self, config: etree._Element | dict, element: GzVisual):
        if isinstance(config, dict):
            return NotImplemented

        visual_xml = etree.SubElement(config, "visual")
        self._writing(visual_xml, element)

    def visit_gz_material(self, config: etree._Element | dict, element: GzMaterial):
        if isinstance(config, dict):
            return NotImplemented

        material_xml = etree.SubElement(config, "material")
        diffuse_xml = etree.SubElement(material_xml, "diffuse")
        if element.diffuse:
            diffuse_xml.text = " ".join(map(str, element.diffuse))

        specular_xml = etree.SubElement(material_xml, "specular")
        if element.specular:
            specular_xml.text = " ".join(map(str, element.specular))

        ambient_xml = etree.SubElement(material_xml, "ambient")
        if element.ambient:
            ambient_xml.text = " ".join(map(str, element.ambient))

        emissive_xml = etree.SubElement(material_xml, "emissive")
        if element.emissive:
            emissive_xml.text = " ".join(map(str, element.emissive))

        lighting = etree.SubElement(material_xml, "lighting")
        if element.lighting is not None:
            lighting.text = str(element.lighting).lower()

        self._writing(material_xml, element)

    def visit_gz_pbr(self, config: etree._Element | dict, element: GzPbr):
        if isinstance(config, dict):
            return NotImplemented

        pbr_xml = etree.SubElement(config, "pbr")
        self._writing(pbr_xml, element)

    def visit_gz_pbr_metal(self, config: etree._Element | dict, element: GzPbrMetal):
        if isinstance(config, dict):
            return NotImplemented

        pbr_metal_xml = etree.SubElement(config, "metal")
        albedo_xml = etree.SubElement(pbr_metal_xml, "albedo_map")
        albedo_xml.text = element.albedo_map

        normal_xml = etree.SubElement(pbr_metal_xml, "normal_map")
        normal_xml.text = element.normal_map

        ambient_occlusion_xml = etree.SubElement(pbr_metal_xml, "ambient_occlusion_map")
        ambient_occlusion_xml.text = element.ambient_occlusion_map

        roughness_xml = etree.SubElement(pbr_metal_xml, "roughness_map")
        roughness_xml.text = element.roughness_map

    def visit_gz_ros(self, config: etree._Element | dict, element: GzRos):
        if isinstance(config, dict):
            return NotImplemented

        ros_xml = etree.SubElement(config, "ros")
        namespace_xml = etree.SubElement(ros_xml, "namespace")
        namespace_xml.text = element.namespace
