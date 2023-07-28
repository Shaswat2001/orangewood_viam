"""
This file registers the OWL robot model with the Python SDK.
"""

from viam.components.arm import Arm
from viam.resource.registry import Registry, ResourceCreatorRegistration

from .owl_robot import OwlRobot

Registry.register_resource_creator(Arm.SUBTYPE, OwlRobot.MODEL, ResourceCreatorRegistration(OwlRobot.new))