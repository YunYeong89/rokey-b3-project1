"""Internal assembly filtering for the existing pose-verified, fixed-joint probe grasp.

The dock and the gripper fix the tool by constraints, not frictional grasp.
Suppress their redundant contacts; retain probe-vs-arm and all scene contacts.
"""
import xml.etree.ElementTree as ET


def group(collision, mask):
    surface = collision.find('surface')
    if surface is None:
        surface = ET.SubElement(collision, 'surface')
    contact = surface.find('contact')
    if contact is None:
        contact = ET.SubElement(surface, 'contact')
    bitmask = contact.find('collide_bitmask')
    if bitmask is None:
        bitmask = ET.SubElement(contact, 'collide_bitmask')
    bitmask.text = str(mask)


def fixed_probe_assembly(robot, rack, probe):
    for link in robot.findall('link'):
        if link.get('name') == 'tool0' or link.get('name', '').startswith('rg2_'):
            for collision in link.findall('collision'):
                group(collision, 1)
    for collision in rack.findall('link/collision'):
        group(collision, 1)
    for collision in probe.findall('link/collision'):
        group(collision, 2)
