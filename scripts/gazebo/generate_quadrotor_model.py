import errno
import os
import shutil
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

CURRENT_DIR = os.path.normpath(os.path.dirname(os.path.realpath(__file__))).split(os.sep)
DESTINATION_DIR = '/' + os.path.join(*CURRENT_DIR[:-2]) + '/gazebo/.gazebo/models/my_quadrotor'
RESOURCES_DIR = '/' + os.path.join(*CURRENT_DIR[:-2]) + '/public'
CONFIG_FILENAME = 'model.config'
MODEL_FILENAME = 'model.sdf'

def createConfigFile(name='', version='0.0', sdf_version='1.6', authors=[],
        maintainer='', description=''):
    # Save locals so that they can be iterated as if they were kwargs
    tags = locals()
    configXml = {}

    # Create xml model
    configXml['root'] = ET.Element('model')
    for t in tags:
        if t == 'authors':
            # List every author as subelement of root
            for a in tags[t]:
                configXml[t] = ET.SubElement(configXml['root'], 'author')
                authorName = ET.SubElement(configXml[t], 'name')
                authorName.text = a[0]
                authorEmail = ET.SubElement(configXml[t], 'email')
                authorEmail.text = a[1]
        elif t == 'sdf_version':
            # Write sdf model filename and attributes
            configXml['sdf'] = ET.SubElement(configXml['root'], 'sdf')
            configXml['sdf'].set('version', tags[t])
            configXml['sdf'].text = 'model.sdf'
        else:
            # Write all elements
            configXml[t] = ET.SubElement(configXml['root'], t)
            configXml[t].text = tags[t]

    # Prettify and write config file
    configFile = open(DESTINATION_DIR + '/' + CONFIG_FILENAME, 'w+')
    configData = ET.tostring(configXml['root'], encoding='utf8').decode('utf8')
    prettyConfigData = minidom.parseString(configData).toprettyxml()
    configFile.write(prettyConfigData)


def createBody(parent=None):
    if parent == None:
        raise ValueError('parent node must be specified')

    # Base link contains base drag, base inertia, base collision, base visual, standoff visuals
    base = ET.SubElement(parent, 'link')
    base.set('name', 'base_link')

    # Base link -> base drag
    baseDrag = ET.SubElement(base, 'velocity_decay')
    baseDragLinear = ET.SubElement(baseDrag, 'linear')
    baseDragLinear.text = '{0:.2f}'.format(0.0) # coef
    baseDragAngular = ET.SubElement(baseDrag, 'angular')
    baseDragAngular.text = '{0:.2f}'.format(0.0) # coef

    # Base link -> base inertial
    baseInertial = ET.SubElement(base, 'inertial')
    baseInertialPose = ET.SubElement(baseInertial, 'pose')
    baseInertialPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, 0, 0) # m, m, m, rad, rad, rad
    baseInertialMass = ET.SubElement(baseInertial, 'mass')
    baseInertialMass.text = '{0:.2f}'.format(1.5) # kg
    baseInertialInertia = ET.SubElement(baseInertial, 'inertia')
    baseInertialInertiaIxx = ET.SubElement(baseInertialInertia, 'ixx')
    baseInertialInertiaIxx.text = '{0:.3f}'.format(0.008)
    baseInertialInertiaIxy = ET.SubElement(baseInertialInertia, 'ixy')
    baseInertialInertiaIxy.text = '{0:.3f}'.format(0)
    baseInertialInertiaIxz = ET.SubElement(baseInertialInertia, 'ixz')
    baseInertialInertiaIxz.text = '{0:.3f}'.format(0)
    baseInertialInertiaIyy = ET.SubElement(baseInertialInertia, 'iyy')
    baseInertialInertiaIyy.text = '{0:.3f}'.format(0.015)
    baseInertialInertiaIyz = ET.SubElement(baseInertialInertia, 'iyz')
    baseInertialInertiaIyz.text = '{0:.3f}'.format(0)
    baseInertialInertiaIzz = ET.SubElement(baseInertialInertia, 'izz')
    baseInertialInertiaIzz.text = '{0:.3f}'.format(0.017)

    # Base link -> base collision
    baseCollision = ET.SubElement(base, 'collision')
    baseCollision.set('name', 'base_link_collision')
    baseCollisionPose = ET.SubElement(baseCollision, 'pose')
    baseCollisionPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, -0.08, 0, 0, 0) # m, m, m, rad, rad, rad
    baseCollisionGeometry = ET.SubElement(baseCollision, 'geometry')
    baseCollisionGeometryBox = ET.SubElement(baseCollisionGeometry, 'box')
    baseCollisionGeometryBoxSize = ET.SubElement(baseCollisionGeometryBox, 'size')
    baseCollisionGeometryBoxSize.text = \
        '{0:.2f} {1:.2f} {2:.2f}'.format(
                0.47, 0.47, 0.23) # m, m, m
    baseCollisionSurface = ET.SubElement(baseCollision, 'surface')
    baseCollisionSurfaceContact = ET.SubElement(baseCollisionSurface, 'contact')
    baseCollisionSurfaceContactOde = ET.SubElement(baseCollisionSurfaceContact, 'ode')
    baseCollisionSurfaceContactOdeMaxVel = ET.SubElement(
            baseCollisionSurfaceContactOde, 'max_vel')
    baseCollisionSurfaceContactOdeMaxVel.text = '{:.2f}'.format(100.0) # m/s
    baseCollisionSurfaceContactOdeMinDepth = ET.SubElement(
            baseCollisionSurfaceContactOde, 'min_depth')
    baseCollisionSurfaceContactOdeMinDepth.text = '{:.3f}'.format(0.001) # m
    baseCollisionSurfaceFriction = ET.SubElement(baseCollisionSurface, 'friction')
    baseCollisionSurfaceFrictionOde = ET.SubElement(baseCollisionSurfaceFriction, 'ode')
    baseCollisionSurfaceFrictionOdeMu = ET.SubElement(
            baseCollisionSurfaceFrictionOde, 'mu')
    baseCollisionSurfaceFrictionOdeMu.text = '{:.2}'.format(1.0)
    baseCollisionSurfaceFrictionOdeMu2 = ET.SubElement(
            baseCollisionSurfaceFrictionOde, 'mu2')
    baseCollisionSurfaceFrictionOdeMu2.text = '{:.2}'.format(1.0)

    # Base link -> base visual
    baseVisual = ET.SubElement(base, 'visual')
    baseVisual.set('name', 'base_link_visual')
    baseVisualGeometry = ET.SubElement(baseVisual, 'geometry')
    baseVisualGeometryMesh = ET.SubElement(baseVisualGeometry, 'mesh')
    baseVisualGeometryMeshUri = ET.SubElement(baseVisualGeometryMesh, 'uri')
    baseVisualGeometryMeshUri.text = 'model://my_quadrotor/meshes/iris.dae'
    baseVisualMaterial = ET.SubElement(baseVisual, 'material')
    baseVisualMaterialScript = ET.SubElement(baseVisualMaterial, 'script')
    baseVisualMaterialScriptName = ET.SubElement(baseVisualMaterialScript, 'name')
    baseVisualMaterialScriptName.text = 'Gazebo/DarkGrey'

    return base


def createStandoffs(parent=None):
    if parent == None:
        raise ValueError('parent node must be specified')

    # Base link -> standoff visuals
    baseFrontLeftLegVisual = ET.SubElement(parent, 'visual')
    baseFrontLeftLegVisual.set('name', 'front_left_leg_visual')
    baseFrontLeftLegVisualPose = ET.SubElement(baseFrontLeftLegVisual, 'pose')
    baseFrontLeftLegVisualPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0.123, 0.22, -0.11, 0, 0, 0) # m, m, m, rad, rad, rad
    baseFrontLeftLegVisualGeometry = ET.SubElement(
            baseFrontLeftLegVisual, 'geometry')
    baseFrontLeftLegVisualGeometryCylinder = ET.SubElement(
            baseFrontLeftLegVisualGeometry, 'cylinder')
    baseFrontLeftLegVisualGeometryCylinderRadius = ET.SubElement(
            baseFrontLeftLegVisualGeometryCylinder, 'radius')
    baseFrontLeftLegVisualGeometryCylinderRadius.text = '{:.3f}'.format(0.005)
    baseFrontLeftLegVisualGeometryCylinderLength = ET.SubElement(
            baseFrontLeftLegVisualGeometryCylinder, 'length')
    baseFrontLeftLegVisualGeometryCylinderLength.text = '{:.3f}'.format(0.17)
    baseFrontLeftLegVisualMaterial = ET.SubElement(
            baseFrontLeftLegVisual, 'material')
    baseFrontLeftLegVisualMaterialScript = ET.SubElement(
            baseFrontLeftLegVisualMaterial, 'script')
    baseFrontLeftLegVisualMaterialScriptName = ET.SubElement(
            baseFrontLeftLegVisualMaterialScript, 'name')
    baseFrontLeftLegVisualMaterialScriptName.text = 'Gazebo/FlatBlack'
    baseFrontLeftLegVisualMaterialScriptUri = ET.SubElement(
            baseFrontLeftLegVisualMaterialScript, 'uri')
    baseFrontLeftLegVisualMaterialScriptUri.text = \
        'file://media/materials/scripts/gazebo.material'

    baseFrontRightLegVisual = ET.SubElement(parent, 'visual')
    baseFrontRightLegVisual.set('name', 'front_right_leg_visual')
    baseFrontRightLegVisualPose = ET.SubElement(baseFrontRightLegVisual, 'pose')
    baseFrontRightLegVisualPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0.123, -0.22, -0.11, 0, 0, 0) # m, m, m, rad, rad, rad
    baseFrontRightLegVisualGeometry = ET.SubElement(
            baseFrontRightLegVisual, 'geometry')
    baseFrontRightLegVisualGeometryCylinder = ET.SubElement(
            baseFrontRightLegVisualGeometry, 'cylinder')
    baseFrontRightLegVisualGeometryCylinderRadius = ET.SubElement(
            baseFrontRightLegVisualGeometryCylinder, 'radius')
    baseFrontRightLegVisualGeometryCylinderRadius.text = '{:.3f}'.format(0.005)
    baseFrontRightLegVisualGeometryCylinderLength = ET.SubElement(
            baseFrontRightLegVisualGeometryCylinder, 'length')
    baseFrontRightLegVisualGeometryCylinderLength.text = '{:.3f}'.format(0.17)
    baseFrontRightLegVisualMaterial = ET.SubElement(
            baseFrontRightLegVisual, 'material')
    baseFrontRightLegVisualMaterialScript = ET.SubElement(
            baseFrontRightLegVisualMaterial, 'script')
    baseFrontRightLegVisualMaterialScriptName = ET.SubElement(
            baseFrontRightLegVisualMaterialScript, 'name')
    baseFrontRightLegVisualMaterialScriptName.text = 'Gazebo/FlatBlack'
    baseFrontRightLegVisualMaterialScriptUri = ET.SubElement(
            baseFrontRightLegVisualMaterialScript, 'uri')
    baseFrontRightLegVisualMaterialScriptUri.text = \
        'file://media/materials/scripts/gazebo.material'

    baseRearLeftLegVisual = ET.SubElement(parent, 'visual')
    baseRearLeftLegVisual.set('name', 'rear_left_leg_visual')
    baseRearLeftLegVisualPose = ET.SubElement(baseRearLeftLegVisual, 'pose')
    baseRearLeftLegVisualPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                -0.140, 0.21, -0.11, 0, 0, 0) # m, m, m, rad, rad, rad
    baseRearLeftLegVisualGeometry = ET.SubElement(
            baseRearLeftLegVisual, 'geometry')
    baseRearLeftLegVisualGeometryCylinder = ET.SubElement(
            baseRearLeftLegVisualGeometry, 'cylinder')
    baseRearLeftLegVisualGeometryCylinderRadius = ET.SubElement(
            baseRearLeftLegVisualGeometryCylinder, 'radius')
    baseRearLeftLegVisualGeometryCylinderRadius.text = '{:.3f}'.format(0.005)
    baseRearLeftLegVisualGeometryCylinderLength = ET.SubElement(
            baseRearLeftLegVisualGeometryCylinder, 'length')
    baseRearLeftLegVisualGeometryCylinderLength.text = '{:.3f}'.format(0.17)
    baseRearLeftLegVisualMaterial = ET.SubElement(
            baseRearLeftLegVisual, 'material')
    baseRearLeftLegVisualMaterialScript = ET.SubElement(
            baseRearLeftLegVisualMaterial, 'script')
    baseRearLeftLegVisualMaterialScriptName = ET.SubElement(
            baseRearLeftLegVisualMaterialScript, 'name')
    baseRearLeftLegVisualMaterialScriptName.text = 'Gazebo/FlatBlack'
    baseRearLeftLegVisualMaterialScriptUri = ET.SubElement(
            baseRearLeftLegVisualMaterialScript, 'uri')
    baseRearLeftLegVisualMaterialScriptUri.text = \
        'file://media/materials/scripts/gazebo.material'

    baseRearRightLegVisual = ET.SubElement(parent, 'visual')
    baseRearRightLegVisual.set('name', 'rear_right_leg_visual')
    baseRearRightLegVisualPose = ET.SubElement(baseRearRightLegVisual, 'pose')
    baseRearRightLegVisualPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                -0.140, -0.21, -0.11, 0, 0, 0) # m, m, m, rad, rad, rad
    baseRearRightLegVisualGeometry = ET.SubElement(
            baseRearRightLegVisual, 'geometry')
    baseRearRightLegVisualGeometryCylinder = ET.SubElement(
            baseRearRightLegVisualGeometry, 'cylinder')
    baseRearRightLegVisualGeometryCylinderRadius = ET.SubElement(
            baseRearRightLegVisualGeometryCylinder, 'radius')
    baseRearRightLegVisualGeometryCylinderRadius.text = '{:.3f}'.format(0.005)
    baseRearRightLegVisualGeometryCylinderLength = ET.SubElement(
            baseRearRightLegVisualGeometryCylinder, 'length')
    baseRearRightLegVisualGeometryCylinderLength.text = '{:.3f}'.format(0.17)
    baseRearRightLegVisualMaterial = ET.SubElement(
            baseRearRightLegVisual, 'material')
    baseRearRightLegVisualMaterialScript = ET.SubElement(
            baseRearRightLegVisualMaterial, 'script')
    baseRearRightLegVisualMaterialScriptName = ET.SubElement(
            baseRearRightLegVisualMaterialScript, 'name')
    baseRearRightLegVisualMaterialScriptName.text = 'Gazebo/FlatBlack'
    baseRearRightLegVisualMaterialScriptUri = ET.SubElement(
            baseRearRightLegVisualMaterialScript, 'uri')
    baseRearRightLegVisualMaterialScriptUri.text = \
        'file://media/materials/scripts/gazebo.material'


# Set rotor link and joint elements
def createRotor(parent=None, rotorNum=0, rotorPose=None, rotorMass=0, rotorInertia=None,
        rotorMesh='', rotorAxisDamping=0, rotorSpringDamper=0,
        rotorColor='Gazebo/DarkGrey'):
    if parent == None:
        raise ValueError('parent node must be specified')

    rotorLink = ET.SubElement(parent, 'link')
    rotorLink.set('name', 'rotor_' + str(rotorNum))

    rotorLinkPose = ET.SubElement(rotorLink, 'pose')
    rotorLinkPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                *rotorPose) # m, m, m, rad, rad, rad

    rotorLinkInertial = ET.SubElement(rotorLink, 'inertial')
    rotorLinkInertialPose = ET.SubElement(rotorLinkInertial, 'pose')
    rotorLinkInertialPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, 0, 0) # m, m, m, rad, rad, rad
    rotorLinkInertialMass = ET.SubElement(rotorLinkInertial, 'mass')
    rotorLinkInertialMass.text = '{:.3f}'.format(rotorMass)
    rotorLinkInertialInertia = ET.SubElement(rotorLinkInertial, 'inertia')
    rotorLinkInertialInertiaIxx = ET.SubElement(rotorLinkInertialInertia, 'ixx')
    rotorLinkInertialInertiaIxx.text = str(rotorInertia[0])
    rotorLinkInertialInertiaIxy = ET.SubElement(rotorLinkInertialInertia, 'ixy')
    rotorLinkInertialInertiaIxy.text = str(rotorInertia[1])
    rotorLinkInertialInertiaIxz = ET.SubElement(rotorLinkInertialInertia, 'ixz')
    rotorLinkInertialInertiaIxz.text = str(rotorInertia[2])
    rotorLinkInertialInertiaIyy = ET.SubElement(rotorLinkInertialInertia, 'iyy')
    rotorLinkInertialInertiaIyy.text = str(rotorInertia[3])
    rotorLinkInertialInertiaIyz = ET.SubElement(rotorLinkInertialInertia, 'iyz')
    rotorLinkInertialInertiaIyz.text = str(rotorInertia[4])
    rotorLinkInertialInertiaIzz = ET.SubElement(rotorLinkInertialInertia, 'izz')
    rotorLinkInertialInertiaIzz.text = str(rotorInertia[5])

    rotorLinkCollision = ET.SubElement(rotorLink, 'collision')
    rotorLinkCollision.set('name', 'rotor_' + str(rotorNum) + '_collision')
    rotorLinkCollisionPose = ET.SubElement(rotorLinkCollision, 'pose')
    rotorLinkCollisionPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, -0, 0) # m, m, m, rad, rad, rad
    rotorLinkCollisionGeometry = ET.SubElement(rotorLinkCollision, 'geometry')
    rotorLinkCollisionGeometryCylinder = ET.SubElement(
            rotorLinkCollisionGeometry, 'cylinder')
    rotorLinkCollisionGeometryCylinderLength = ET.SubElement(
            rotorLinkCollisionGeometryCylinder, 'length')
    rotorLinkCollisionGeometryCylinderLength.text = '{:.3f}'.format(0.005)
    rotorLinkCollisionGeometryCylinderRadius = ET.SubElement(
            rotorLinkCollisionGeometryCylinder, 'radius')
    rotorLinkCollisionGeometryCylinderRadius.text = '{:.3f}'.format(0.1)
    rotorLinkCollisionSurface = ET.SubElement(rotorLinkCollision, 'surface')
    rotorLinkCollisionSurfaceContact = ET.SubElement(
            rotorLinkCollisionSurface, 'contact')
    rotorLinkCollisionSurfaceContactOde = ET.SubElement(
            rotorLinkCollisionSurfaceContact, 'ode')
    rotorLinkCollisionSurfaceFriction = ET.SubElement(
            rotorLinkCollisionSurface, 'friction')
    rotorLinkCollisionSurfaceFrictionOde = ET.SubElement(
            rotorLinkCollisionSurfaceFriction, 'ode')
    rotorLinkVisual = ET.SubElement(rotorLink, 'visual')
    rotorLinkVisual.set('name', 'rotor_' + str(rotorNum) + '_visual')
    rotorLinkVisualPose = ET.SubElement(rotorLinkVisual, 'pose')
    rotorLinkVisualPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, -0, 0) # m, m, m, rad, rad, rad
    rotorLinkVisualGeometry = ET.SubElement(rotorLinkVisual, 'geometry')
    rotorLinkVisualGeometryMesh = ET.SubElement(rotorLinkVisualGeometry, 'mesh')
    rotorLinkVisualGeometryMeshScale = ET.SubElement(rotorLinkVisualGeometryMesh, 'scale')
    rotorLinkVisualGeometryMeshScale.text = '{0:.1f} {1:.1f} {2:.1f}'.format(1, 1, 1)
    rotorLinkVisualGeometryMeshUri = ET.SubElement(rotorLinkVisualGeometryMesh, 'uri')
    rotorLinkVisualGeometryMeshUri.text = '{0}'.format(rotorMesh)
    rotorLinkVisualMaterial = ET.SubElement(rotorLinkVisual, 'material')
    rotorLinkVisualMaterialScript = ET.SubElement(
            rotorLinkVisualMaterial, 'script')
    rotorLinkVisualMaterialScriptName = ET.SubElement(
            rotorLinkVisualMaterialScript, 'name')
    rotorLinkVisualMaterialScriptName.text = rotorColor
    rotorLinkVisualMaterialScriptUri = ET.SubElement(
            rotorLinkVisualMaterialScript, 'uri')
    rotorLinkVisualMaterialScriptUri.text = '__default__'

    rotorLinkGravity = ET.SubElement(rotorLink, 'gravity')
    rotorLinkGravity.text = '{}'.format(1)

    rotorLinkVelocity_decay = ET.SubElement(rotorLink, 'velocity_decay')

    rotorLinkSelf_collide = ET.SubElement(rotorLink, 'self_collide')
    rotorLinkSelf_collide.text = '{0}'.format(0)

    rotorJoint = ET.SubElement(parent, 'joint')
    rotorJoint.set('name', 'rotor_' + str(rotorNum) + '_joint')
    rotorJoint.set('type', 'revolute')
    rotorJointChild = ET.SubElement(rotorJoint, 'child')
    rotorJointChild.text = 'rotor_' + str(rotorNum)
    rotorJointParent = ET.SubElement(rotorJoint, 'parent')
    rotorJointParent.text = 'base_link'
    rotorJointAxis = ET.SubElement(rotorJoint, 'axis')
    rotorJointAxisXyz = ET.SubElement(rotorJointAxis, 'xyz')
    rotorJointAxisXyz.text = '{0} {1} {2}'.format(0, 0, 1)
    rotorJointAxisLimit = ET.SubElement(rotorJointAxis, 'limit')
    rotorJointAxisLimitLower = ET.SubElement(rotorJointAxisLimit, 'lower')
    rotorJointAxisLimitLower.text = '{0}'.format(-1e16)
    rotorJointAxisLimitUpper = ET.SubElement(rotorJointAxisLimit, 'upper')
    rotorJointAxisLimitUpper.text = '{0}'.format(1e16)
    rotorJointAxisDynamics = ET.SubElement(rotorJointAxis, 'dynamics')
    rotorJointAxisDynamicsDamping = ET.SubElement(
            rotorJointAxisDynamics, 'damping')
    rotorJointAxisDynamicsDamping.text = '{:.3f}'.format(rotorAxisDamping)
    rotorJointAxisUse_parent_model_frame = ET.SubElement(
            rotorJointAxis, 'use_parent_model_frame')
    rotorJointAxisUse_parent_model_frame.text = '{0}'.format(1)
    rotorJointPhysics = ET.SubElement(rotorJoint, 'physics')
    rotorJointPhysicsOde = ET.SubElement(rotorJointPhysics, 'ode')
    rotorJointPhysicsOdeImplicit_spring_damper = ET.SubElement(
            rotorJointPhysicsOde, 'implicit_spring_damper')
    rotorJointPhysicsOdeImplicit_spring_damper.text = '{0}'.format(1)

    rotorPlugin_b0 = ET.SubElement(parent, 'plugin')
    rotorPlugin_b0.set('name', 'rotor_' + str(rotorNum) + '_blade_1')
    rotorPlugin_b0.set('filename', 'libLiftDragPlugin.so')
    rotorPlugin_b0A0 = ET.SubElement(rotorPlugin_b0, 'a0')
    rotorPlugin_b0A0.text = '{:.1f}'.format(0.3)
    rotorPlugin_b0Alpha_stall = ET.SubElement(rotorPlugin_b0, 'alpha_stall')
    rotorPlugin_b0Alpha_stall.text = '{:.4f}'.format(1.4)
    rotorPlugin_b0Cla = ET.SubElement(rotorPlugin_b0, 'cla')
    rotorPlugin_b0Cla.text = '{:.4f}'.format(4.25)
    rotorPlugin_b0Cda = ET.SubElement(rotorPlugin_b0, 'cda')
    rotorPlugin_b0Cda.text = '{:.4f}'.format(0.1)
    rotorPlugin_b0Cma = ET.SubElement(rotorPlugin_b0, 'cma')
    rotorPlugin_b0Cma.text = '{:.4f}'.format(0.0)
    rotorPlugin_b0Cla_stall = ET.SubElement(rotorPlugin_b0, 'cla_stall')
    rotorPlugin_b0Cla_stall.text = '{:.4f}'.format(-0.025)
    rotorPlugin_b0Cda_stall = ET.SubElement(rotorPlugin_b0, 'cda_stall')
    rotorPlugin_b0Cda_stall.text = '{:.4f}'.format(0.0)
    rotorPlugin_b0Cma_stall = ET.SubElement(rotorPlugin_b0, 'cma_stall')
    rotorPlugin_b0Cma.text = '{:.4f}'.format(0.0)
    rotorPlugin_b0Area = ET.SubElement(rotorPlugin_b0, 'area')
    rotorPlugin_b0Area.text = '{:.4f}'.format(0.002)
    rotorPlugin_b0Air_density = ET.SubElement(rotorPlugin_b0, 'air_density')
    rotorPlugin_b0Cp = ET.SubElement(rotorPlugin_b0, 'cp')
    rotorPlugin_b0Cp.text = '{0:.4f} {1:.4f} {2:.4f}'.format(0.084, 0, 0)
    rotorPlugin_b0Forward = ET.SubElement(rotorPlugin_b0, 'forward')
    if rotorNum == 0 or rotorNum == 1:
        rotorPlugin_b0Forward.text = '{0} {1} {2}'.format(0, 1, 0)
    else:
        rotorPlugin_b0Forward.text = '{0} {1} {2}'.format(0, -1, 0)
    rotorPlugin_b0Upward = ET.SubElement(rotorPlugin_b0, 'upward')
    rotorPlugin_b0Upward.text = '{0} {1} {2}'.format(0, 0, 1)
    rotorPlugin_b0Link_name = ET.SubElement(rotorPlugin_b0, 'link_name')
    rotorPlugin_b0Link_name.text = 'my_quadrotor::rotor_{}'.format(rotorNum)

    rotorPlugin_b0 = ET.SubElement(parent, 'plugin')
    rotorPlugin_b0.set('name', 'rotor_' + str(rotorNum) + '_blade_2')
    rotorPlugin_b0.set('filename', 'libLiftDragPlugin.so')
    rotorPlugin_b0A0 = ET.SubElement(rotorPlugin_b0, 'a0')
    rotorPlugin_b0A0.text = '{:.1f}'.format(0.3)
    rotorPlugin_b0Alpha_stall = ET.SubElement(rotorPlugin_b0, 'alpha_stall')
    rotorPlugin_b0Alpha_stall.text = '{:.4f}'.format(1.4)
    rotorPlugin_b0Cla = ET.SubElement(rotorPlugin_b0, 'cla')
    rotorPlugin_b0Cla.text = '{:.4f}'.format(4.25)
    rotorPlugin_b0Cda = ET.SubElement(rotorPlugin_b0, 'cda')
    rotorPlugin_b0Cda.text = '{:.4f}'.format(0.1)
    rotorPlugin_b0Cma = ET.SubElement(rotorPlugin_b0, 'cma')
    rotorPlugin_b0Cma.text = '{:.4f}'.format(0.0)
    rotorPlugin_b0Cla_stall = ET.SubElement(rotorPlugin_b0, 'cla_stall')
    rotorPlugin_b0Cla_stall.text = '{:.4f}'.format(-0.025)
    rotorPlugin_b0Cda_stall = ET.SubElement(rotorPlugin_b0, 'cda_stall')
    rotorPlugin_b0Cda_stall.text = '{:.4f}'.format(0.0)
    rotorPlugin_b0Cma_stall = ET.SubElement(rotorPlugin_b0, 'cma_stall')
    rotorPlugin_b0Cma.text = '{:.4f}'.format(0.0)
    rotorPlugin_b0Area = ET.SubElement(rotorPlugin_b0, 'area')
    rotorPlugin_b0Area.text = '{:.4f}'.format(0.002)
    rotorPlugin_b0Air_density = ET.SubElement(rotorPlugin_b0, 'air_density')
    rotorPlugin_b0Cp = ET.SubElement(rotorPlugin_b0, 'cp')
    rotorPlugin_b0Cp.text = '{0:.4f} {1:.4f} {2:.4f}'.format(-0.084, 0, 0)
    rotorPlugin_b0Forward = ET.SubElement(rotorPlugin_b0, 'forward')
    if rotorNum == 0 or rotorNum == -1:
        rotorPlugin_b0Forward.text = '{0} {1} {2}'.format(0, -1, 0)
    else:
        rotorPlugin_b0Forward.text = '{0} {1} {2}'.format(0, 1, 0)
    rotorPlugin_b0Upward = ET.SubElement(rotorPlugin_b0, 'upward')
    rotorPlugin_b0Upward.text = '{0} {1} {2}'.format(0, 0, 1)
    rotorPlugin_b0Link_name = ET.SubElement(rotorPlugin_b0, 'link_name')
    rotorPlugin_b0Link_name.text = 'my_quadrotor::rotor_{}'.format(rotorNum)

    return rotorLink,rotorJoint


def createGroundTruthSensor(parent=None, linkName='', jointName='', mass=0,
        inertia=None, pluginName='', pluginFileName='', pluginTopicName=''):
    if parent == None:
        raise ValueError('parent node must be specified')

    gtLink = ET.SubElement(parent, 'link')
    gtLink.set('name', linkName)
    gtLinkPose = ET.SubElement(gtLink, 'pose')
    gtLinkPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, 0, 0) # m, m, m, rad, rad, rad
    gtLinkInertial = ET.SubElement(gtLink, 'inertial')
    gtLinkInertialPose = ET.SubElement(gtLinkInertial, 'pose')
    gtLinkInertialPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, 0, 0) # m, m, m, rad, rad, rad
    gtLinkInertialMass = ET.SubElement(gtLinkInertial, 'mass')
    gtLinkInertialMass.text = '{:.2f}'.format(mass)
    gtLinkInertialInertia = ET.SubElement(gtLinkInertial, 'inertia')
    gtLinkInertialInertiaIxx = ET.SubElement(gtLinkInertialInertia, 'ixx')
    gtLinkInertialInertiaIxx.text = '{:.4f}'.format(inertia[0])
    gtLinkInertialInertiaIxy = ET.SubElement(gtLinkInertialInertia, 'ixy')
    gtLinkInertialInertiaIxy.text = '{:.4f}'.format(inertia[1])
    gtLinkInertialInertiaIxz = ET.SubElement(gtLinkInertialInertia, 'ixz')
    gtLinkInertialInertiaIxz.text = '{:.4f}'.format(inertia[2])
    gtLinkInertialInertiaIyy = ET.SubElement(gtLinkInertialInertia, 'iyy')
    gtLinkInertialInertiaIyy.text = '{:.4f}'.format(inertia[3])
    gtLinkInertialInertiaIyz = ET.SubElement(gtLinkInertialInertia, 'iyz')
    gtLinkInertialInertiaIyz.text = '{:.4f}'.format(inertia[4])
    gtLinkInertialInertiaIzz = ET.SubElement(gtLinkInertialInertia, 'izz')
    gtLinkInertialInertiaIzz.text = '{:.4f}'.format(inertia[5])
    gtLinkSensor = ET.SubElement(gtLink, 'sensor')
    gtLinkSensor.set('name', 'mygt')
    gtLinkSensor.set('type', 'gps')
    gtLinkSensorPose = ET.SubElement(gtLinkSensor, 'pose')
    gtLinkSensorPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, 0, 0) # m, m, m, rad, rad, rad
    gtLinkSensorUpdate_rate = ET.SubElement(gtLinkSensor, 'update_rate')
    gtLinkSensorUpdate_rate.text = '{:.1}'.format(100.0)
    gtLinkSensorAlways_on = ET.SubElement(gtLinkSensor, 'always_on')
    gtLinkSensorAlways_on.text = 'true'
    gtLinkSensorGps = ET.SubElement(gtLinkSensor, 'gps')
    gtLinkSensorGpsPosition_sensing = ET.SubElement(gtLinkSensorGps, 'position_sensing')
    gtLinkSensorGpsPosition_sensingHorizontal = ET.SubElement(
            gtLinkSensorGpsPosition_sensing, 'horizontal')
    gtLinkSensorGpsPosition_sensingHorizontalNoise = ET.SubElement(
            gtLinkSensorGpsPosition_sensingHorizontal, 'noise')
    gtLinkSensorGpsPosition_sensingHorizontalNoise.set('type', 'none')

    gtJoint = ET.SubElement(parent, 'joint')
    gtJoint.set('name', jointName)
    gtJoint.set('type', 'revolute')
    gtJointChild = ET.SubElement(gtJoint, 'child')
    gtJointChild.text = linkName
    gtJointParent = ET.SubElement(gtJoint, 'parent')
    gtJointParent.text = 'base_link'
    gtJointAxis = ET.SubElement(gtJoint, 'axis')
    gtJointAxisXyz = ET.SubElement(gtJointAxis, 'xyz')
    gtJointAxisXyz.text = '{0} {1} {2}'.format(0, 0, 1)
    gtJointAxisLimit = ET.SubElement(gtJointAxis, 'limit')
    gtJointAxisLimitLower = ET.SubElement(gtJointAxisLimit, 'lower')
    gtJointAxisLimitLower.text = '{0}'.format(0)
    gtJointAxisLimitUpper = ET.SubElement(gtJointAxisLimit, 'upper')
    gtJointAxisLimitUpper.text = '{0}'.format(0)
    gtJointAxisLimitEffort = ET.SubElement(gtJointAxisLimit, 'effort')
    gtJointAxisLimitEffort.text = '{0}'.format(0)
    gtJointAxisLimitVelocity = ET.SubElement(gtJointAxisLimit, 'velocity')
    gtJointAxisLimitVelocity.text = '{0}'.format(0)
    gtJointAxisDynamics = ET.SubElement(gtJointAxis, 'dynamics')
    gtJointAxisDynamicsDamping = ET.SubElement(
            gtJointAxisDynamics, 'damping')
    gtJointAxisDynamicsDamping.text = '{:.1f}'.format(1.0)
    gtJointAxisUse_parent_model_frame = ET.SubElement(
            gtJointAxis, 'use_parent_model_frame')
    gtJointAxisUse_parent_model_frame.text = '{}'.format(1)
    gtJointPhysics = ET.SubElement(gtJoint, 'physics')
    gtJointPhysicsOde = ET.SubElement(gtJointPhysics, 'ode')
    gtJointPhysicsOdeImplicit_spring_damper = ET.SubElement(
            gtJointPhysicsOde, 'implicit_spring_damper')
    gtJointPhysicsOdeImplicit_spring_damper.text = '{}'.format(1)

    return gtLink, gtJoint


def createImu(parent=None, linkName='', jointName='', mass=0,
        inertia=None):
    if parent == None:
        raise ValueError('parent node must be specified')

    imuLink = ET.SubElement(parent, 'link')
    imuLink.set('name', linkName)
    imuLinkInertial = ET.SubElement(imuLink, 'inertial')
    imuLinkInertialPose = ET.SubElement(imuLinkInertial, 'pose')
    imuLinkInertialPose.text = \
        '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f}'.format(
                0, 0, 0, 0, 0, 0) # m, m, m, rad, rad, rad
    imuLinkInertialMass = ET.SubElement(imuLinkInertial, 'mass')
    imuLinkInertialMass.text = '{:.2f}'.format(mass)
    imuLinkInertialInertia = ET.SubElement(imuLinkInertial, 'inertia')
    imuLinkInertialInertiaIxx = ET.SubElement(imuLinkInertialInertia, 'ixx')
    imuLinkInertialInertiaIxx.text = '{:.5f}'.format(inertia[0])
    imuLinkInertialInertiaIxy = ET.SubElement(imuLinkInertialInertia, 'ixy')
    imuLinkInertialInertiaIxy.text = '{:.5f}'.format(inertia[1])
    imuLinkInertialInertiaIxz = ET.SubElement(imuLinkInertialInertia, 'ixz')
    imuLinkInertialInertiaIxz.text = '{:.5f}'.format(inertia[2])
    imuLinkInertialInertiaIyy = ET.SubElement(imuLinkInertialInertia, 'iyy')
    imuLinkInertialInertiaIyy.text = '{:.5f}'.format(inertia[3])
    imuLinkInertialInertiaIyz = ET.SubElement(imuLinkInertialInertia, 'iyz')
    imuLinkInertialInertiaIyz.text = '{:.5f}'.format(inertia[4])
    imuLinkInertialInertiaIzz = ET.SubElement(imuLinkInertialInertia, 'izz')
    imuLinkInertialInertiaIzz.text = '{:.5f}'.format(inertia[5])
    imuLinkSensor = ET.SubElement(imuLink, 'sensor')
    imuLinkSensor.set('name', 'imu_sensor')
    imuLinkSensor.set('type', 'imu')
    imuLinkSensorPose = ET.SubElement(imuLinkSensor, 'pose')
    imuLinkSensorPose.text = '{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f} {5:.6f} '.format(
            0, 0, 0, 3.141593, 0, 0)
    imuLinkSensorAlways_on = ET.SubElement(imuLinkSensor, 'always_on')
    imuLinkSensorAlways_on.text = '{}'.format(1)
    imuLinkSensorUpdate_rate = ET.SubElement(imuLinkSensor, 'update_rate')
    imuLinkSensorUpdate_rate.text = '{:.1f}'.format(1000.0)

    imuJoint = ET.SubElement(parent, 'joint')
    imuJoint.set('name', jointName)
    imuJoint.set('type', 'revolute')
    imuJointChild = ET.SubElement(imuJoint, 'child')
    imuJointChild.text = linkName
    imuJointParent = ET.SubElement(imuJoint, 'parent')
    imuJointParent.text = 'base_link'
    imuJointAxis = ET.SubElement(imuJoint, 'axis')
    imuJointAxisXyz = ET.SubElement(imuJointAxis, 'xyz')
    imuJointAxisXyz.text = '{0} {1} {2}'.format(0, 0, 1)
    imuJointAxisLimit = ET.SubElement(imuJointAxis, 'limit')
    imuJointAxisLimitLower = ET.SubElement(imuJointAxisLimit, 'lower')
    imuJointAxisLimitLower.text = '{0}'.format(0)
    imuJointAxisLimitUpper = ET.SubElement(imuJointAxisLimit, 'upper')
    imuJointAxisLimitUpper.text = '{0}'.format(0)
    imuJointAxisLimitEffort = ET.SubElement(imuJointAxisLimit, 'effort')
    imuJointAxisLimitEffort.text = '{0}'.format(0)
    imuJointAxisLimitVelocity = ET.SubElement(imuJointAxisLimit, 'velocity')
    imuJointAxisLimitVelocity.text = '{0}'.format(0)
    imuJointAxisDynamics = ET.SubElement(imuJointAxis, 'dynamics')
    imuJointAxisDynamicsDamping = ET.SubElement(
            imuJointAxisDynamics, 'damping')
    imuJointAxisDynamicsDamping.text = '{:.1f}'.format(1.0)
    imuJointAxisUse_parent_model_frame = ET.SubElement(
            imuJointAxis, 'use_parent_model_frame')
    imuJointAxisUse_parent_model_frame.text = '{}'.format(1)
    imuJointPhysics = ET.SubElement(imuJoint, 'physics')
    imuJointPhysicsOde = ET.SubElement(imuJointPhysics, 'ode')
    imuJointPhysicsOdeImplicit_spring_damper = ET.SubElement(
            imuJointPhysicsOde, 'implicit_spring_damper')
    imuJointPhysicsOdeImplicit_spring_damper.text = '{}'.format(1)

    return imuLink, imuJoint


def createCamera(parent=None):
    if parent == None:
        raise ValueError('parent node must be specified')


def createPlugin(parent=None, pluginName='', pluginFileName=''):
    if parent == None:
        raise ValueError('parent node must be specified')

    plugin = ET.SubElement(parent, 'plugin')
    plugin.set('filename', pluginFileName)
    plugin.set('name', pluginName)

    pluginGt = ET.SubElement(plugin, 'gt')
    pluginGtUpdate_rate = ET.SubElement(pluginGt, 'update_rate')
    pluginGtUpdate_rate.text = '{:.1}'.format(100.0)
    pluginGtFrame = ET.SubElement(pluginGt, 'frame')
    pluginGtFrame.text = 'world'

    pluginImu = ET.SubElement(plugin, 'imu')
    pluginImuUpdate_rate = ET.SubElement(pluginImu, 'update_rate')
    pluginImuUpdate_rate.text = '{:.1}'.format(100.0)
    pluginImuFrame = ET.SubElement(pluginImu, 'frame')
    pluginImuFrame.text = 'world'

    pluginRotor0 = ET.SubElement(plugin, 'rotor0')

    pluginRotor1 = ET.SubElement(plugin, 'rotor1')

    pluginRotor2 = ET.SubElement(plugin, 'rotor2')

    pluginRotor3 = ET.SubElement(plugin, 'rotor3')

    pluginCamera = ET.SubElement(plugin, 'camera')

    pluginControl = ET.SubElement(plugin, 'control')

    return plugin


def createModelFile(modelName='my_quadrotor', sdf_version='1.6'):
    # Create model base elements
    rootEl = ET.Element('sdf')
    rootEl.set('version', sdf_version)
    modelEl = ET.SubElement(rootEl, 'model')
    modelEl.set('name', modelName)

    # Set model pose
    modelPose = ET.SubElement(modelEl, 'pose')
    modelPose.text = \
        '{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f} {5:.6f}'.format(
                0, 0, 0.194923, 0, 0, 0) # m, m, m, rad, rad, rad

    # Create model body
    bodyBaseEl = createBody(parent=modelEl)

    # Create model standoffs
    standoffEls = createStandoffs(parent=bodyBaseEl)

    grountTruthEls = createGroundTruthSensor(parent=modelEl,
            linkName='groundtruth_link',
            jointName='groundtruth_joint',
            mass=0.15,
            inertia=[0.0001, 0, 0, 0.0002, 0, 0.0002])

    # Create model imu
    imuEls = createImu(parent=modelEl,
            linkName='imu_link',
            jointName='imu_joint',
            mass=0.15,
            inertia=[0.00001, 0, 0, 0.00002, 0, 0.00002])

    # Create model camera
    cameraEls = createCamera(parent=modelEl)

    # Create model rotors
    rotor0El = createRotor(parent=modelEl, rotorNum=0,
            rotorPose=[0.13, -0.22, 0.023, 0, -0, 0],
            rotorMass=0.025,
            rotorInertia=[9.75e-6, 0, 0, 0.000166704, 0, 0.000167604],
            rotorMesh='model://iris_with_standoffs/meshes/iris_prop_ccw.dae',
            rotorAxisDamping=0.004,
            rotorSpringDamper=1,
            rotorColor='Gazebo/Blue')
    rotor1El = createRotor(parent=modelEl, rotorNum=1,
            rotorPose=[-0.13, 0.2, 0.023, 0, -0, 0],
            rotorMass=0.025,
            rotorInertia=[9.75e-6, 0, 0, 0.000166704, 0, 0.000167604],
            rotorMesh='model://iris_with_standoffs/meshes/iris_prop_ccw.dae',
            rotorAxisDamping=0.004,
            rotorSpringDamper=1,
            rotorColor='Gazebo/DarkGrey')
    rotor2El = createRotor(parent=modelEl, rotorNum=2,
            rotorPose=[0.13, 0.22, 0.023, 0, -0, 0],
            rotorMass=0.025,
            rotorInertia=[9.75e-6, 0, 0, 0.000166704, 0, 0.000167604],
            rotorMesh='model://iris_with_standoffs/meshes/iris_prop_cw.dae',
            rotorAxisDamping=0.004,
            rotorSpringDamper=1,
            rotorColor='Gazebo/Blue')
    rotor3El = createRotor(parent=modelEl, rotorNum=3,
            rotorPose=[-0.13, -0.2, 0.023, 0, -0, 0],
            rotorMass=0.025,
            rotorInertia=[9.75e-6, 0, 0, 0.000166704, 0, 0.000167604],
            rotorMesh='model://iris_with_standoffs/meshes/iris_prop_cw.dae',
            rotorAxisDamping=0.004,
            rotorSpringDamper=1,
            rotorColor='Gazebo/DarkGrey')

    pluginEl = createPlugin(parent=modelEl,
            pluginName='my_quadrotor',
            pluginFileName='libmy_quadrotor_plugin.so')

    staticEl = ET.SubElement(modelEl, 'static')
    staticEl.text = '{}'.format(0)

    # Prettify and write model file
    modelFile = open(DESTINATION_DIR + '/' + MODEL_FILENAME, 'w+')
    modelData = ET.tostring(rootEl, encoding='utf8').decode('utf8')
    prettyModelData = minidom.parseString(modelData).toprettyxml()
    modelFile.write(prettyModelData)


if __name__ == '__main__':
    # Create output directory for quadrotor model
    for output_dir in [DESTINATION_DIR, DESTINATION_DIR + '/meshes']:
        try:
            os.makedirs(output_dir)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

    # Move meshes to destination directory
    for src_dir, dirs, files in os.walk(RESOURCES_DIR + '/meshes/iris'):
        for f in files:
            shutil.copy2(os.path.join(src_dir, f),
                    DESTINATION_DIR + '/meshes/' + f)

    createConfigFile(name='my_quadrotor', version='0.0', sdf_version='1.6',
            authors=[('Eric Solomon', 'errcsool@engineer.com')], maintainer='errcsool@engineer.com',
            description='Quadrotor model built from Python')

    createModelFile(modelName='my_quadrotor', sdf_version='1.6')

