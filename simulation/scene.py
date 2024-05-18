"""Replay recorded joint positions through the same Drake model used for dynamics."""

import numpy as np
from PIL import Image
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    CameraInfo,
    ClippingRange,
    ColorRenderCamera,
    DiagramBuilder,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    LightParameter,
)
from simulation.dynamics.drake_plant import populate


class ReplayScene:
    def __init__(self, parameters, width, height):
        builder = DiagramBuilder()
        self.plant, graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        populate(self.plant, parameters, visuals=True)

        vtk = RenderEngineVtkParams()
        vtk.default_clear_color = [0.93, 0.95, 0.96]
        vtk.lights = [
            LightParameter(
                type="directional",
                frame="world",
                direction=[-0.6, 0.55, -1.0],
                intensity=0.9,
            ),
            LightParameter(
                type="point", frame="world", position=[-0.7, -0.8, 0.4], intensity=0.45
            ),
        ]
        graph.AddRenderer("studio", MakeRenderEngineVtk(vtk))

        self.diagram = builder.Build()
        root = self.diagram.CreateDefaultContext()
        self.ctx = self.plant.GetMyMutableContextFromRoot(root)
        self.graph_ctx = graph.GetMyContextFromRoot(root)
        self.root = root
        self.query_port = graph.get_query_output_port()

        size = parameters["rod_length"] + parameters["arm_length"]
        target = np.array([0.0, 0.0, -0.05]) * size / 0.5
        eye = np.array([1.18, -1.75, 0.87]) * size
        forward = target - eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        rotation = RotationMatrix(
            np.column_stack((right, np.cross(forward, right), forward))
        )
        self.pose = RigidTransform(rotation, eye)
        self.camera = ColorRenderCamera(
            RenderCameraCore(
                "studio",
                CameraInfo(width, height, 0.76),
                ClippingRange(0.01, 10.0),
                RigidTransform(),
            ),
            False,
        )

    def position(self, a, q):
        self.plant.SetPositions(self.ctx, [a, q])

    def image(self):
        query = self.query_port.Eval(self.graph_ctx)
        pixels = query.RenderColorImage(
            self.camera, SceneGraph.world_frame_id(), self.pose
        )
        return Image.fromarray(pixels.data).convert("RGB")
