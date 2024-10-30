# SPDX-License-Identifier: BSD-3-Clause

# flake8: noqa F401
from collections.abc import Callable

import numpy as np

from vendeeglobe import (
    Checkpoint,
    Heading,
    Instructions,
    Location,
    Vector,
    config,
)
from vendeeglobe.utils import distance_on_surface


class Bot:
    """
    This is the ship-controlling bot that will be instantiated for the competition.
    """

    def __init__(self):
        self.team = "TeamName"  # This is your team name
        # This is the course that the ship has to follow
        self.course = [
            Checkpoint(latitude=20.03, longitude=-67.31, radius=50),
            Checkpoint(latitude=18.04, longitude=-67.85, radius=50),
            Checkpoint(latitude=12.02, longitude=-81.39, radius=50),
            Checkpoint(latitude=06.33, longitude=-78.8, radius=50),
            Checkpoint(latitude=-14.64, longitude=-168.46, radius=50.0), #c1
            Checkpoint(latitude=-36.95, longitude=162.56, radius=50.0),
            Checkpoint(latitude=-36.8, longitude=112.8, radius=50.0),
            Checkpoint(latitude=-24.24, longitude=72.67, radius=50.0), #c2
            Checkpoint(latitude=-37.2, longitude=19.24, radius=50.0), 
            Checkpoint(latitude=11.72, longitude=-32.1, radius=50.0), 
            Checkpoint(latitude=45.72, longitude=-10.88, radius=50.0), 
            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            ),
        ]

    def run(
        self,
        t: float,
        dt: float,
        longitude: float,
        latitude: float,
        heading: float,
        speed: float,
        vector: np.ndarray,
        forecast: Callable,
        world_map: Callable,
    ) -> Instructions:
        """
        This is the method that will be called at every time step to get the
        instructions for the ship.

        Parameters
        ----------
        t:
            The current time in hours.
        dt:
            The time step in hours.
        longitude:
            The current longitude of the ship.
        latitude:
            The current latitude of the ship.
        heading:
            The current heading of the ship.
        speed:
            The current speed of the ship.
        vector:
            The current heading of the ship, expressed as a vector.
        forecast:
            Method to query the weather forecast for the next 5 days.
            Example:
            current_position_forecast = forecast(
                latitudes=latitude, longitudes=longitude, times=0
            )
        world_map:
            Method to query map of the world: 1 for sea, 0 for land.
            Example:
            current_position_terrain = world_map(
                latitudes=latitude, longitudes=longitude
            )

        Returns
        -------
        instructions:
            A set of instructions for the ship. This can be:
            - a Location to go to
            - a Heading to point to
            - a Vector to follow
            - a number of degrees to turn Left
            - a number of degrees to turn Right

            Optionally, a sail value between 0 and 1 can be set.
        """
        # Initialize the instructions
        instructions = Instructions()

        # TODO: Remove this, it's only for testing =================
        current_position_forecast = forecast(
            latitudes=latitude, longitudes=longitude, times=0
        )
        current_position_terrain = world_map(latitudes=latitude, longitudes=longitude)
        # ===========================================================

        # Go through all checkpoints and find the next one to reach
       # Go through all checkpoints and find the next one to reach
        for ch in self.course:
            # Compute the distance to the checkpoint
            dist = distance_on_surface(
                longitude1=longitude,
                latitude1=latitude,
                longitude2=ch.longitude,
                latitude2=ch.latitude,
            )
            
            # turn around if stuck, dont run within the first hour
            if t > 1:
                if speed == 0:
                    print("reversing as we are stuck")
                    prev_heading = heading
                    print("old heading", prev_heading)
                    instructions.heading = prev_heading - 180 % 360 #turn around, keep the number within 0 - 360 
                    print("new heading", heading)
            
            # Consider slowing down if the checkpoint is close
            jump = dt * np.linalg.norm(speed)
            if dist < 2.0 * ch.radius + jump: # only slow down on the last checkpoint
                instructions.sail = 1.0 #min(ch.radius / jump, 1)
            else:
                instructions.sail = 1.0
            # Check if the checkpoint has been reached
            if dist < ch.radius:
                ch.reached = True
            if not ch.reached:
                instructions.location = Location(
                    longitude=ch.longitude, latitude=ch.latitude
                )
                break

        return instructions