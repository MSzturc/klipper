# Axis Twist Compensation
#
# Copyright (C) 2022  Jeremy Tan <jeremytkw98@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math, logging
from . import manual_probe, bed_mesh, probe


DEFAULT_SAMPLE_COUNT = 3
DEFAULT_SPEED = 50.
DEFAULT_HORIZONTAL_MOVE_Z = 5.


class AxisTwistCompensation:
    def __init__(self, config):
        # Initialize AxisTwistCompensation with printer configuration.
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Load values from configuration.
        self.horizontal_move_z = config.getfloat('horizontal_move_z', DEFAULT_HORIZONTAL_MOVE_Z)
        self.speed = config.getfloat('speed', DEFAULT_SPEED)
        self.calibrate_start_x = config.getfloat('calibrate_start_x', default=None)
        self.calibrate_end_x = config.getfloat('calibrate_end_x', default=None)
        self.calibrate_y = config.getfloat('calibrate_y', default=None)
        self.z_compensations = config.getlists('z_compensations', default=[], parser=float)
        self.compensation_start_x = config.getfloat('compensation_start_x', default=None)
        self.compensation_end_x = config.getfloat('compensation_end_x', default=None)

        self.calibrate_start_y = config.getfloat('calibrate_start_y', default=None)
        self.calibrate_end_y = config.getfloat('calibrate_end_y', default=None)
        self.calibrate_x = config.getfloat('calibrate_x', default=None)
        self.compensation_start_y = config.getfloat('compensation_start_y', default=None)
        self.compensation_end_y = config.getfloat('compensation_end_y', default=None)
        self.zy_compensations = config.getlists('zy_compensations', default=[], parser=float)

        # Initialize calibrater
        self.calibrater = Calibrater(self, config)

        # Register event handler
        self.printer.register_event_handler("probe:update_results", self._update_z_compensation_value)

        logging.info("AxisTwistCompensation initialized with provided configuration.")

    def _update_z_compensation_value(self, pos):
        # Update Z compensation value based on the current position.
        logging.debug(f"Updating Z compensation value for position: {pos}")
        if self.z_compensations:
            pos[2] += self._get_interpolated_z_compensation(
                pos[0], self.z_compensations,
                self.compensation_start_x,
                self.compensation_end_x
            )
            logging.debug(f"Z compensation applied: {pos[2]:.6f}")

        if self.zy_compensations:
            pos[2] += self._get_interpolated_z_compensation(
                pos[1], self.zy_compensations,
                self.compensation_start_y,
                self.compensation_end_y
            )
            logging.debug(f"ZY compensation applied: {pos[2]:.6f}")

    def _get_interpolated_z_compensation(self, coord, z_compensations, comp_start, comp_end):
        # Interpolate Z compensation value for a given coordinate.
        logging.debug(f"Interpolating Z compensation for coord: {coord}, range: ({comp_start}, {comp_end})")
        sample_count = len(z_compensations)
        spacing = (comp_end - comp_start) / (sample_count - 1)
        interpolate_t = (coord - comp_start) / spacing
        interpolate_i = int(math.floor(interpolate_t))
        interpolate_i = bed_mesh.constrain(interpolate_i, 0, sample_count - 2)
        interpolate_t -= interpolate_i

        interpolated_z_compensation = bed_mesh.lerp(
            interpolate_t, z_compensations[interpolate_i],
            z_compensations[interpolate_i + 1]
        )
        logging.debug(f"Interpolated Z compensation: {interpolated_z_compensation:.6f}")
        return interpolated_z_compensation

    def clear_compensations(self, axis=None):
        # Clear compensation values for the specified axis.
        if axis is None:
            self.z_compensations = []
            self.zy_compensations = []
            logging.info("All compensation values cleared.")
        elif axis == 'X':
            self.z_compensations = []
            logging.info("X-axis compensation values cleared.")
        elif axis == 'Y':
            self.zy_compensations = []
            logging.info("Y-axis compensation values cleared.")

class Calibrater:
    def __init__(self, compensation, config):
        # Initialize Calibrater with compensation and configuration.
        self.compensation = compensation
        self.printer = compensation.printer
        self.gcode = self.printer.lookup_object('gcode')
        self.probe = None

        # Probe settings initialized to None until available.
        self.lift_speed, self.probe_x_offset, self.probe_y_offset, _ = None, None, None, None
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.speed = compensation.speed
        self.horizontal_move_z = compensation.horizontal_move_z
        self.x_start_point = (compensation.calibrate_start_x, compensation.calibrate_y)
        self.x_end_point = (compensation.calibrate_end_x, compensation.calibrate_y)
        self.y_start_point = (compensation.calibrate_x, compensation.calibrate_start_y)
        self.y_end_point = (compensation.calibrate_x, compensation.calibrate_end_y)
        self.results = None
        self.current_point_index = None
        self.gcmd = None
        self.configname = config.get_name()

        # Register G-code handlers.
        self._register_gcode_handlers()
        logging.info("Calibrater initialized for compensation module.")

    def _handle_connect(self):
        # Handle printer connection and set probe parameters.
        self.probe = self.printer.lookup_object('probe', None)
        if self.probe is None:
            config = self.printer.lookup_object('configfile')
            raise config.error("AXIS_TWIST_COMPENSATION requires [probe] to be defined")
        self.lift_speed = self.probe.get_probe_params()['lift_speed']
        self.probe_x_offset, self.probe_y_offset, _ = self.probe.get_offsets()
        logging.info("Probe parameters set: lift_speed=%s, x_offset=%s, y_offset=%s", 
                     self.lift_speed, self.probe_x_offset, self.probe_y_offset)

    def _register_gcode_handlers(self):
        # Register G-code commands for calibration.
        self.gcode.register_command(
            'AXIS_TWIST_COMPENSATION_CALIBRATE',
            self.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE,
            desc=self.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help
        )
        logging.info("Registered G-code command: AXIS_TWIST_COMPENSATION_CALIBRATE")

    cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help = """
    Performs the X twist calibration wizard.
    Measure Z probe offset at N points along the X axis,
    and calculate X twist compensation.
    """

    def cmd_AXIS_TWIST_COMPENSATION_CALIBRATE(self, gcmd):
        # Log the start of the calibration command
        logging.info("Starting AXIS_TWIST_COMPENSATION_CALIBRATE command.")
        
        # Store the G-code command for later use
        self.gcmd = gcmd
        
        # Parse the sample count, axis, and auto-calibration flag from the G-code command
        sample_count = gcmd.get_int('SAMPLE_COUNT', DEFAULT_SAMPLE_COUNT)
        axis = gcmd.get('AXIS', None)
        auto = gcmd.get('AUTO', False)

        # Check for conflicting axis and auto-calibration options
        if axis is not None and auto:
            error_msg = "Cannot use both 'AXIS' and 'AUTO' at the same time."
            logging.error(error_msg)
            raise self.gcmd.error(error_msg)

        # If auto-calibration is enabled, start it and exit the function
        if auto:
            logging.info("Starting auto-calibration with %d samples.", sample_count)
            self._start_autocalibration(sample_count)
            return

        # Default to 'X' axis if no axis is specified
        if axis is None:
            axis = 'X'

        # Validate the sample count
        if sample_count < 2:
            error_msg = "SAMPLE_COUNT to probe must be at least 2."
            logging.error(error_msg)
            raise self.gcmd.error(error_msg)

        # Initialize the list of nozzle points
        nozzle_points = []

        # Calculate nozzle points for the X-axis
        if axis == 'X':
            logging.info("Starting X-axis calibration with %d sample points.", sample_count)
            self.compensation.clear_compensations('X')

            # Validate the required calibration points for the X-axis
            if not all([self.x_start_point[0], self.x_end_point[0], self.x_start_point[1]]):
                error_msg = ("AXIS_TWIST_COMPENSATION for X axis requires "
                            "calibrate_start_x, calibrate_end_x, and calibrate_y to be defined.")
                logging.error(error_msg)
                raise self.gcmd.error(error_msg)

            # Calculate the range and interval for the X-axis
            start_point = self.x_start_point
            end_point = self.x_end_point
            x_axis_range = end_point[0] - start_point[0]
            interval_dist = x_axis_range / (sample_count - 1)

            # Generate the X-axis nozzle points
            for i in range(sample_count):
                x = start_point[0] + i * interval_dist
                y = start_point[1]
                nozzle_points.append((x, y))
            logging.debug("X-axis nozzle points: %s", nozzle_points)

        # Calculate nozzle points for the Y-axis
        elif axis == 'Y':
            logging.info("Starting Y-axis calibration with %d sample points.", sample_count)
            self.compensation.clear_compensations('Y')

            # Validate the required calibration points for the Y-axis
            if not all([self.y_start_point[0], self.y_end_point[0], self.y_start_point[1]]):
                error_msg = ("AXIS_TWIST_COMPENSATION for Y axis requires "
                            "calibrate_start_y, calibrate_end_y, and calibrate_x to be defined.")
                logging.error(error_msg)
                raise self.gcmd.error(error_msg)

            # Calculate the range and interval for the Y-axis
            start_point = self.y_start_point
            end_point = self.y_end_point
            y_axis_range = end_point[1] - start_point[1]
            interval_dist = y_axis_range / (sample_count - 1)

            # Generate the Y-axis nozzle points
            for i in range(sample_count):
                x = start_point[0]
                y = start_point[1] + i * interval_dist
                nozzle_points.append((x, y))
            logging.debug("Y-axis nozzle points: %s", nozzle_points)

        # Handle invalid axis specification
        else:
            error_msg = "AXIS_TWIST_COMPENSATION_CALIBRATE: Invalid axis."
            logging.error(error_msg)
            raise self.gcmd.error(error_msg)

        # Calculate probe points based on nozzle points and probe offsets
        probe_points = self._calculate_probe_points(nozzle_points, self.probe_x_offset, self.probe_y_offset)
        logging.debug("Calculated probe points: %s", probe_points)

        # Verify no manual probe is currently in progress
        manual_probe.verify_no_manual_probe(self.printer)
        logging.info("No manual probe in progress. Proceeding with calibration.")

        # Initialize calibration variables
        self.current_point_index = 0
        self.results = []
        self.current_axis = axis

        # Start the calibration process
        logging.info("Beginning calibration process for axis: %s", axis)
        self._calibration(probe_points, nozzle_points, interval_dist)

    def _calculate_corrections(self, coordinates):
        # Extracting x, y, and z values from coordinates
        # Coordinates are expected to be in the format [(x1, y1, z1), (x2, y2, z2), ...]
        x_coords = [coord[0] for coord in coordinates]
        y_coords = [coord[1] for coord in coordinates]
        z_coords = [coord[2] for coord in coordinates]

        # Logging the extracted coordinates for debugging purposes
        logging.debug(f"Extracted x coordinates: {x_coords}")
        logging.debug(f"Extracted y coordinates: {y_coords}")
        logging.debug(f"Extracted z coordinates: {z_coords}")

        # Calculate the desired point (average of all corner points in z)
        # For a general case, we should extract the unique combinations of corner points
        z_corners = [z_coords[i] for i, coord in enumerate(coordinates)
                        if (coord[0] in [x_coords[0], x_coords[-1]])
                        and (coord[1] in [y_coords[0], y_coords[-1]])]

        # Log the corner z-values used for calculating the desired point
        logging.debug(f"Z-values at corners: {z_corners}")

        z_desired = sum(z_corners) / len(z_corners)

        # Log the calculated desired z-value
        logging.debug(f"Calculated desired z-value: {z_desired}")

        # Calculate average deformation per axis
        unique_x_coords = sorted(set(x_coords))
        unique_y_coords = sorted(set(y_coords))

        # Log the unique x and y coordinates for debugging
        logging.debug(f"Unique x coordinates: {unique_x_coords}")
        logging.debug(f"Unique y coordinates: {unique_y_coords}")

        avg_z_x = []
        for x in unique_x_coords:
            indices = [i for i, coord in enumerate(coordinates)
                        if coord[0] == x]
            avg_z = sum(z_coords[i] for i in indices) / len(indices)
            avg_z_x.append(avg_z)
            
            # Log the calculation for each unique x
            logging.debug(f"For x = {x}: indices = {indices}, avg_z = {avg_z}")

        avg_z_y = []
        for y in unique_y_coords:
            indices = [i for i, coord in enumerate(coordinates)
                        if coord[1] == y]
            avg_z = sum(z_coords[i] for i in indices) / len(indices)
            avg_z_y.append(avg_z)
            
            # Log the calculation for each unique y
            logging.debug(f"For y = {y}: indices = {indices}, avg_z = {avg_z}")

        # Calculate corrections to reach the desired point
        x_corrections = [z_desired - avg for avg in avg_z_x]
        y_corrections = [z_desired - avg for avg in avg_z_y]

        # Log the calculated corrections
        logging.debug(f"X-axis corrections: {x_corrections}")
        logging.debug(f"Y-axis corrections: {y_corrections}")

        return x_corrections, y_corrections


    def _start_autocalibration(self, sample_count):
        # Check if start and end points are properly defined
        if not all([
                self.x_start_point[0],
                self.x_end_point[0],
                self.y_start_point[0],
                self.y_end_point[0]
                ]):
                logging.error("Calibration start and end points are not properly defined.")
                raise self.gcmd.error(
                    """AXIS_TWIST_COMPENSATION_AUTOCALIBRATE requires
                    calibrate_start_x, calibrate_end_x, calibrate_start_y
                    and calibrate_end_y to be defined
                    """
                    )

        # Check for valid sample_count
        if sample_count is None or sample_count < 2:
            logging.error("Invalid SAMPLE_COUNT provided: must be at least 2.")
            raise self.gcmd.error(
                "SAMPLE_COUNT to probe must be at least 2")

        # Verify no other manual probe is in progress
        logging.info("Verifying no manual probe is in progress.")
        manual_probe.verify_no_manual_probe(self.printer)

        # Clear the current compensation configuration
        logging.info("Clearing current compensation configuration.")
        self.compensation.clear_compensations()

        min_x = self.x_start_point[0]
        max_x = self.x_end_point[0]
        min_y = self.y_start_point[1]
        max_y = self.y_end_point[1]

        # Calculate x positions
        interval_x = (max_x - min_x) / (sample_count - 1)
        xps = [min_x + interval_x * i for i in range(sample_count)]
        logging.debug(f"Calculated x positions: {xps}")

        # Calculate points array
        interval_y = (max_y - min_y) / (sample_count - 1)
        flip = False

        points = []
        for i in range(sample_count):
            for j in range(sample_count):
                if not flip:
                    idx = j
                else:
                    idx = sample_count - 1 - j
                points.append([xps[i], min_y + interval_y * idx])
            flip = not flip
        logging.debug(f"Generated probing points: {points}")

        # Calculate the points to put the nozzle at, and probe
        probe_points = []

        for i in range(len(points)):
            x = points[i][0] - self.probe_x_offset
            y = points[i][1] - self.probe_y_offset
            probe_point = [x, y, self._auto_calibration((x, y))[2]]
            probe_points.append(probe_point)
            logging.debug(f"Probed point: {probe_point}")

        # Calculate corrections
        x_corr, y_corr = self._calculate_corrections(probe_points)

        x_corr_str = ', '.join(["{:.6f}".format(x) for x in x_corr])
        y_corr_str = ', '.join(["{:.6f}".format(x) for x in y_corr])

        logging.debug(f"X-axis corrections: {x_corr}")
        logging.debug(f"Y-axis corrections: {y_corr}")

        # Finalize configuration
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.configname, 'z_compensations', x_corr_str)
        configfile.set(self.configname, 'compensation_start_x',
                    self.x_start_point[0])
        configfile.set(self.configname, 'compensation_end_x',
                    self.x_end_point[0])
        configfile.set(self.configname, 'zy_compensations', y_corr_str)
        configfile.set(self.configname, 'compensation_start_y',
                    self.y_start_point[1])
        configfile.set(self.configname, 'compensation_end_y',
                    self.y_end_point[1])

        self.gcode.respond_info(
            "AXIS_TWIST_COMPENSATION state has been saved "
            "for the current session. The SAVE_CONFIG command will "
            "update the printer config file and restart the printer.")

        # Output result
        self.gcmd.respond_info(
            "AXIS_TWIST_COMPENSATION_AUTOCALIBRATE: Calibration complete: ")
        self.gcmd.respond_info("\n".join(map(str, [x_corr, y_corr])), log=False)
        logging.info("Calibration complete.")


    def _auto_calibration(self, probe_point):
        # Move to safe Z height to prevent collisions
        logging.info("Moving to safe Z height before probing.")
        self._move_helper((None, None, self.horizontal_move_z))

        # Move to the specified probe point
        logging.info(f"Moving to probe point: {probe_point}.")
        self._move_helper((probe_point[0], probe_point[1], None))

        # Perform probing operation
        logging.info("Starting probing operation.")
        pos = probe.run_single_probe(self.probe, self.gcmd)
        logging.debug(f"Probe position returned: {pos}")

        # Move back to safe Z height
        logging.info("Moving back to safe Z height after probing.")
        self._move_helper((None, None, self.horizontal_move_z))

        return pos

    def _calculate_probe_points(self, nozzle_points, probe_x_offset, probe_y_offset):
            # Calculate probe points by offsetting nozzle points
            logging.info("Calculating probe points with offsets.")
            probe_points = []
            for point in nozzle_points:
                x = point[0] - probe_x_offset
                y = point[1] - probe_y_offset
                probe_points.append((x, y))
                logging.debug(f"Calculated probe point: ({x}, {y})")
            return probe_points

    def _move_helper(self, target_coordinates, override_speed=None):
            # Prepare target coordinates
            target_coordinates = (
                target_coordinates[0], target_coordinates[1], None
            ) if len(target_coordinates) == 2 else target_coordinates
            toolhead = self.printer.lookup_object('toolhead')
            speed = self.speed if target_coordinates[2] is None else self.lift_speed
            speed = override_speed if override_speed is not None else speed

            logging.info(f"Moving to target coordinates: {target_coordinates} at speed: {speed}")
            toolhead.manual_move(target_coordinates, speed)

    def _calibration(self, probe_points, nozzle_points, interval):
            # Begin calibration process
            logging.info("Starting calibration process.")
            self.gcmd.respond_info(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Probing point %d of %d" % (
                    self.current_point_index + 1, len(probe_points)))

            # Move to safe Z height
            logging.info("Moving to safe Z height before probing.")
            self._move_helper((None, None, self.horizontal_move_z))

            # Move to the current probe point
            current_probe_point = probe_points[self.current_point_index]
            logging.info(f"Moving to current probe point: {current_probe_point}.")
            self._move_helper((current_probe_point[0], current_probe_point[1], None))

            # Perform the probing operation
            logging.info("Performing probing operation.")
            pos = probe.run_single_probe(self.probe, self.gcmd)
            self.current_measured_z = pos[2]
            logging.debug(f"Measured Z position: {self.current_measured_z}")

            # Move back to safe Z height
            logging.info("Moving back to safe Z height after probing.")
            self._move_helper((None, None, self.horizontal_move_z))

            # Move the nozzle to the probe point
            current_nozzle_point = nozzle_points[self.current_point_index]
            logging.info(f"Moving nozzle to current probe point: {current_nozzle_point}.")
            self._move_helper(current_nozzle_point)

            # Start manual probe
            logging.info("Starting manual probe operation.")
            manual_probe.ManualProbeHelper(
                self.printer, self.gcmd,
                self._manual_probe_callback_factory(probe_points, nozzle_points, interval))

    def _manual_probe_callback_factory(self, probe_points, nozzle_points, interval):
            is_end = self.current_point_index == len(probe_points) - 1

            def callback(kin_pos):
                if kin_pos is None:
                    logging.warning("Manual probe cancelled.")
                    self.gcmd.respond_info(
                        "AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, calibration aborted")
                    return
                z_offset = self.current_measured_z - kin_pos[2]
                logging.debug(f"Calculated Z offset: {z_offset}")
                self.results.append(z_offset)

                if is_end:
                    logging.info("Finalizing calibration as this is the last probe point.")
                    self._finalize_calibration()
                else:
                    logging.info("Proceeding to the next probe point.")
                    self.current_point_index += 1
                    self._calibration(probe_points, nozzle_points, interval)

            return callback

    def _finalize_calibration(self):
            logging.info("Finalizing calibration process.")

            avg = sum(self.results) / len(self.results)
            self.results = [avg - x for x in self.results]

            logging.debug(f"Calculated average Z offset: {avg}")
            logging.debug(f"Normalized results: {self.results}")

            configfile = self.printer.lookup_object('configfile')
            values_as_str = ', '.join(["{:.6f}".format(x) for x in self.results])

            if self.current_axis == 'X':
                logging.info("Saving X-axis calibration results.")
                configfile.set(self.configname, 'z_compensations', values_as_str)
                configfile.set(self.configname, 'compensation_start_x', self.x_start_point[0])
                configfile.set(self.configname, 'compensation_end_x', self.x_end_point[0])

                self.compensation.z_compensations = self.results
                self.compensation.compensation_start_x = self.x_start_point[0]
                self.compensation.compensation_end_x = self.x_end_point[0]

            elif self.current_axis == 'Y':
                logging.info("Saving Y-axis calibration results.")
                configfile.set(self.configname, 'zy_compensations', values_as_str)
                configfile.set(self.configname, 'compensation_start_y', self.y_start_point[1])
                configfile.set(self.configname, 'compensation_end_y', self.y_end_point[1])

                self.compensation.zy_compensations = self.results
                self.compensation.compensation_start_y = self.y_start_point[1]
                self.compensation.compensation_end_y = self.y_end_point[1]

            self.gcode.respond_info(
                "AXIS_TWIST_COMPENSATION state has been saved "
                "for the current session. The SAVE_CONFIG command will "
                "update the printer config file and restart the printer.")

            logging.info("Calibration process completed successfully.")

            self.gcmd.respond_info(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, "
                "offsets: %s, mean z_offset: %f" % (self.results, avg))



# klipper's entry point using [axis_twist_compensation] section in printer.cfg
def load_config(config):
    return AxisTwistCompensation(config)
