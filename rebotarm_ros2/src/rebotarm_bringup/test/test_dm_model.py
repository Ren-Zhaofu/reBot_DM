from pathlib import Path
from xml.etree import ElementTree


def test_dm_urdf_has_complete_chain_and_meshes() -> None:
    package = Path(__file__).parents[1]
    urdf = package / "description/urdf/reBot-DevArm_fixend.urdf"
    root = ElementTree.parse(urdf).getroot()

    links = {element.attrib["name"] for element in root.findall("link")}
    assert links == {
        "base_footprint",
        "base_link",
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6",
        "end_link",
    }
    movable_joints = [
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib["type"] != "fixed"
    ]
    assert movable_joints == [f"joint{index}" for index in range(1, 7)]

    mesh_uris = {mesh.attrib["filename"] for mesh in root.findall(".//mesh")}
    expected = {
        f"package://rebotarm_bringup/description/meshes/{name}"
        for name in (
            "base_link.STL",
            "link1.STL",
            "link2.STL",
            "link3.STL",
            "link4.STL",
            "link5.STL",
            "link6.STL",
            "end_link.STL",
        )
    }
    assert mesh_uris == expected
    for uri in mesh_uris:
        relative = uri.removeprefix("package://rebotarm_bringup/")
        assert (package / relative).is_file()
