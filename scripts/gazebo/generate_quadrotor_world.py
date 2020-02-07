import errno
import os
import shutil
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

CURRENT_DIR = os.path.normpath(os.path.dirname(os.path.realpath(__file__))).split(os.sep)
DESTINATION_DIR = '/' + os.path.join(*CURRENT_DIR[:-2]) + '/gazebo/.gazebo/worlds/'
RESOURCES_DIR = '/' + os.path.join(*CURRENT_DIR[:-2]) + '/public'
WORLD_FILENAME = 'my_quadrotor.world'

def createWorldFile(worldName='generic', sdfVersion='1.6'):
    rootEl = ET.Element('sdf')
    rootEl.set('version', sdfVersion)
    worldEl = ET.SubElement(rootEl, 'world')
    worldEl.set('name', worldName)

    worldPhysics = ET.SubElement(worldEl, 'physics')
    worldPhysics.set('type', 'ode')
    worldPhysicsOde = ET.SubElement(worldPhysics, 'ode')
    worldPhysicsOdeSolver = ET.SubElement(worldPhysicsOde, 'solver')
    worldPhysicsOdeSolverType = ET.SubElement(worldPhysicsOdeSolver, 'type')
    worldPhysicsOdeSolverType.text = 'quick'
    worldPhysicsOdeSolverIters = ET.SubElement(worldPhysicsOdeSolver, 'iters')
    worldPhysicsOdeSolverIters.text = '{}'.format(100)
    worldPhysicsOdeSolverSor = ET.SubElement(worldPhysicsOdeSolver, 'sor')
    worldPhysicsOdeSolverSor.text = '{:.1f}'.format(1.0)
    worldPhysicsOdeConstraints = ET.SubElement(worldPhysicsOde, 'constraints')
    worldPhysicsOdeConstraintsCfm = ET.SubElement(worldPhysicsOdeConstraints, 'cfm')
    worldPhysicsOdeConstraintsCfm.text = '{:.1f}'.format(0.0)
    worldPhysicsOdeConstraintsErp = ET.SubElement(worldPhysicsOdeConstraints, 'erp')
    worldPhysicsOdeConstraintsErp.text = '{:.1f}'.format(0.9)
    worldPhysicsOdeConstraintsContact_max_correcting_vel = ET.SubElement(
            worldPhysicsOdeConstraints, 'contact_max_correcting_vel')
    worldPhysicsOdeConstraintsContact_max_correcting_vel.text = '{:.1f}'.format(0.1)
    worldPhysicsOdeConstraintsContact_surface_layer = ET.SubElement(
            worldPhysicsOdeConstraints, 'contact_surface_layer')
    worldPhysicsOdeConstraintsContact_surface_layer.text = '{:.1f}'.format(0.0)
    worldPhysicsReal_time_update_rate = ET.SubElement(
            worldPhysics, 'real_time_update_rate')
    worldPhysicsReal_time_update_rate.text = '{}'.format(0)
    worldPhysicsMax_step_size = ET.SubElement(
            worldPhysics, 'max_step_size')
    worldPhysicsMax_step_size.text = '{:.4f}'.format(0.0025)

    worldInclude = ET.SubElement(worldEl, 'include')
    worldIncludeUri = ET.SubElement(worldInclude, 'uri')
    worldIncludeUri.text = 'model://sun'

    worldModel = ET.SubElement(worldEl, 'model')
    worldModel.set('name', 'ground_plane')
    worldModelStatic = ET.SubElement(worldModel, 'static')
    worldModelStatic.text = 'true'

    worldModelLink = ET.SubElement(worldModel, 'link')
    worldModelLink.set('name', 'link')
    worldModelLinkCollision = ET.SubElement(worldModelLink, 'collision')
    worldModelLinkCollision.set('name', 'collision')
    worldModelLinkCollisionGeometry = ET.SubElement(
            worldModelLinkCollision, 'geometry')
    worldModelLinkCollisionGeometryPlane = ET.SubElement(
            worldModelLinkCollisionGeometry, 'plane')
    worldModelLinkCollisionGeometryPlaneNormal = ET.SubElement(
            worldModelLinkCollisionGeometryPlane, 'normal')
    worldModelLinkCollisionGeometryPlaneNormal.text = \
            '{0} {1} {2}'.format(0, 0, 1)
    worldModelLinkCollisionGeometryPlaneSize = ET.SubElement(
            worldModelLinkCollisionGeometryPlane, 'size')
    worldModelLinkCollisionGeometryPlaneSize.text = \
            '{0} {1}'.format(5000, 5000)
    worldModelLinkCollisionSurface = ET.SubElement(
            worldModelLinkCollision, 'surface')
    worldModelLinkCollisionSurfaceFriction = ET.SubElement(
            worldModelLinkCollisionSurface, 'friction')
    worldModelLinkCollisionSurfaceFrictionOde = ET.SubElement(
            worldModelLinkCollisionSurfaceFriction, 'ode')
    worldModelLinkCollisionSurfaceFrictionOdeMu = ET.SubElement(
            worldModelLinkCollisionSurfaceFrictionOde, 'mu')
    worldModelLinkCollisionSurfaceFrictionOdeMu.text = \
            '{:.1f}'.format(1)
    worldModelLinkCollisionSurfaceFrictionOdeMu2 = ET.SubElement(
            worldModelLinkCollisionSurfaceFrictionOde, 'mu2')
    worldModelLinkCollisionSurfaceFrictionOdeMu2.text = \
            '{:.1f}'.format(1)
    worldModelLinkVisual = ET.SubElement(worldModelLink, 'visual')
    worldModelLinkVisual.set('name', 'runway')
    worldModelLinkVisualPose = ET.SubElement(
            worldModelLinkVisual, 'pose')
    worldModelLinkVisualPose.text = \
            '{0:.4f} {1:.4f} {2:.4f} {3:.4f} {4:.4f} {5:.4f} '.format(
                    0, 0, 0.005, 0, 0, -1.5707)
    worldModelLinkVisualCast_shadows = ET.SubElement(
            worldModelLinkVisual, 'cast_shadows')
    worldModelLinkVisualCast_shadows.text = 'false'
    worldModelLinkVisualGeometry = ET.SubElement(
            worldModelLinkVisual, 'geometry')
    worldModelLinkVisualGeometryPlane = ET.SubElement(
            worldModelLinkVisualGeometry, 'plane')
    worldModelLinkVisualGeometryPlaneNormal = ET.SubElement(
            worldModelLinkVisualGeometryPlane, 'normal')
    worldModelLinkVisualGeometryPlaneNormal.text = \
            '{0} {1} {2}'.format(0, 0, 1)
    worldModelLinkVisualGeometryPlaneSize = ET.SubElement(
            worldModelLinkVisualGeometryPlane, 'size')
    worldModelLinkVisualGeometryPlaneSize.text = \
            '{0} {1}'.format(1829, 45)
    worldModelLinkVisualMaterial = ET.SubElement(
            worldModelLinkVisual, 'material')
    worldModelLinkVisualMaterialScript = ET.SubElement(
            worldModelLinkVisualMaterial, 'script')
    worldModelLinkVisualMaterialScriptUri = ET.SubElement(
            worldModelLinkVisualMaterialScript, 'uri')
    worldModelLinkVisualMaterialScriptUri.text = \
            'file://media/materials/scripts/gazebo.material'
    worldModelLinkVisualMaterialScriptName = ET.SubElement(
            worldModelLinkVisualMaterialScript, 'name')
    worldModelLinkVisualMaterialScriptName.text = \
            'Gazebo/Runway'

    worldModelLinkVisual = ET.SubElement(worldModelLink, 'visual')
    worldModelLinkVisual.set('name', 'grass')
    worldModelLinkVisualPose = ET.SubElement(
            worldModelLinkVisual, 'pose')
    worldModelLinkVisualPose.text = \
            '{0:.4f} {1:.4f} {2:.4f} {3:.4f} {4:.4f} {5:.4f} '.format(
                    0, 0, -0.1, 0, 0, 0)
    worldModelLinkVisualCast_shadows = ET.SubElement(
            worldModelLinkVisual, 'cast_shadows')
    worldModelLinkVisualCast_shadows.text = 'false'
    worldModelLinkVisualGeometry = ET.SubElement(
            worldModelLinkVisual, 'geometry')
    worldModelLinkVisualGeometryPlane = ET.SubElement(
            worldModelLinkVisualGeometry, 'plane')
    worldModelLinkVisualGeometryPlaneNormal = ET.SubElement(
            worldModelLinkVisualGeometryPlane, 'normal')
    worldModelLinkVisualGeometryPlaneNormal.text = \
            '{0} {1} {2}'.format(0, 0, 1)
    worldModelLinkVisualGeometryPlaneSize = ET.SubElement(
            worldModelLinkVisualGeometryPlane, 'size')
    worldModelLinkVisualGeometryPlaneSize.text = \
            '{0} {1}'.format(5000, 5000)
    worldModelLinkVisualMaterial = ET.SubElement(
            worldModelLinkVisual, 'material')
    worldModelLinkVisualMaterialScript = ET.SubElement(
            worldModelLinkVisualMaterial, 'script')
    worldModelLinkVisualMaterialScriptUri = ET.SubElement(
            worldModelLinkVisualMaterialScript, 'uri')
    worldModelLinkVisualMaterialScriptUri.text = \
            'file://media/materials/scripts/gazebo.material'
    worldModelLinkVisualMaterialScriptName = ET.SubElement(
            worldModelLinkVisualMaterialScript, 'name')
    worldModelLinkVisualMaterialScriptName.text = \
            'Gazebo/Grass'

    worldModel2 = ET.SubElement(worldEl, 'model')
    worldModel2.set('name', 'my_quadrotor')
    worldModel2Pose = ET.SubElement(worldModel2, 'pose')
    worldModel2Pose.text = \
            '{0:.4f} {1:.4f} {2:.4f} {3:.4f} {4:.4f} {5:.4f} '.format(
                    0, 0, 0, 0, 0, 0)
    worldModel2Include = ET.SubElement(worldModel2, 'include')
    worldModel2IncludeUri = ET.SubElement(worldModel2Include, 'uri')
    worldModel2IncludeUri.text = 'model://my_quadrotor'
    worldModel2IncludePose = ET.SubElement(worldModel2Include, 'pose')
    worldModel2IncludePose.text = \
            '{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f} {5:.6f} '.format(
                    0, 0, 0.194923, 0, 0, 0)

    # Prettify and write world file
    worldFile = open(DESTINATION_DIR + '/' + WORLD_FILENAME, 'w+')
    worldData = ET.tostring(rootEl, encoding='utf8').decode('utf8')
    prettyWorldData = minidom.parseString(worldData).toprettyxml()
    worldFile.write(prettyWorldData)


if __name__ == '__main__':
    # Create output directory for quadrotor model
    try:
        os.makedirs(DESTINATION_DIR)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

    createWorldFile(worldName='my_quadrotor', sdfVersion='1.6')
