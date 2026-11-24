"""Force-controlled scan of one part of the real vehicle (repair_car real_scan)."""

MESH_CONTROLLERS = ('mesh-height',)
SCAN_CONTROLLERS = (*MESH_CONTROLLERS, 'default')
BOX_CONTROLLERS = ('mesh-height', 'default')
