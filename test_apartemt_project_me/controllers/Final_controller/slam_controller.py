import math
import numpy as np 

# def uncertent_add(distance, angle, sigma):
	# mean = np.array([distance, angle])
	# covariance = np.diag(sigma**2)
	# distance, angle = np.random.multivarite_normal(mean, covariance)
	# distance= max(distance,0)
	# angle =max(angle,0)
	# return[distance,angle]


# class LaserSensor:
	# """docstring for LaserSensor"""
	# def __init__(self, Rnage, map, uncertenty):
		# self.Rnage = Rnage
		# self.sigma= np.array([uncertenty[0], uncertenty[1]])
		# self.position = (0,0)
		# self.map = map

import time

# Basic params
_DEFAULT_MAP_QUALITY         = 50 # out of 255
_DEFAULT_HOLE_WIDTH_MM       = 600

# Random mutation hill-climbing (RMHC) params
_DEFAULT_SIGMA_XY_MM         = 100
_DEFAULT_SIGMA_THETA_DEGREES = 20
_DEFAULT_MAX_SEARCH_ITER     = 1000

# CoreSLAM class ------------------------------------------------------------------------------------------------------

class CoreSLAM(object):
    
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
        map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):
        
        self.map_quality = map_quality
        self.hole_width_mm = hole_width_mm   
        
        # Store laser for later
        self.laser = laser
        
        # Initialize a scan for computing distance to map, and one for updating map
        self.scan_for_distance = pybreezyslam(laser, 1)
        self.scan_for_mapbuild = pybreezyslam(laser, 3)
                
        # Initialize the map 
        self.map = pybreezyslam.Map(map_size_pixels, map_size_meters)
                
    def update(self, scans_mm, pose_change, scan_angles_degrees=None, should_update_map=True):
        '''
        Updates the scan and odometry, and calls the the implementing class's _updateMapAndPointcloud method with
        the specified pose change.
         
        scan_mm is a list of Lidar scan values, whose count is specified in the scan_size 
        attribute of the Laser object passed to the CoreSlam constructor
        pose_change is a tuple (dxy_mm, dtheta_degrees, dt_seconds) computed from odometry
        scan_angles_degrees is an optional list of angles corresponding to the distances in scans_mm
        should_update_map flags for whether you want to update the map
        '''

        # Convert pose change (dxy,dtheta,dt) to velocities (dxy/dt, dtheta/dt) for scan update
        velocity_factor = (1 / pose_change[2])  if (pose_change[2] > 0) else 0 # units => units/sec
        dxy_mm_dt = pose_change[0] * velocity_factor  
        dtheta_degrees_dt = pose_change[1] * velocity_factor
        velocities = (dxy_mm_dt, dtheta_degrees_dt)

        # Build a scan for computing distance to map, and one for updating map 
        self._scan_update(self.scan_for_mapbuild, scans_mm, velocities, scan_angles_degrees)
        self._scan_update(self.scan_for_distance, scans_mm, velocities, scan_angles_degrees)

        # Implementing class updates map and pointcloud
        self._updateMapAndPointcloud(pose_change[0], pose_change[1], should_update_map)
        
    def getmap(self, mapbytes):
        '''
        Fills bytearray mapbytes with current map pixels, where bytearray length is square of map size passed
        to CoreSLAM.__init__().
        '''
        self.map.get(mapbytes)
        
        
    def setmap(self, mapbytes):
        '''
        Sets current map pixels to values in bytearray, where bytearray length is square of map size passed
        to CoreSLAM.__init__().
        '''
        self.map.set(mapbytes)

    def __str__(self):
        
        return 'CoreSLAM: %s \n          map quality = %d / 255 \n          hole width = %7.0f mm' % \
               (str(self.map), self.map_quality, self.hole_width_mm)
                
    def __repr__(self):
        
         return self.__str__()

        
    def _scan_update(self, scan, scans_distances_mm, velocities, scan_angles_degrees):

        scan.update(scans_mm=scans_distances_mm, hole_width_mm=self.hole_width_mm, 
                velocities=velocities, scan_angles_degrees=scan_angles_degrees)
        
        
# SinglePositionSLAM class ---------------------------------------------------------------------------------------------

class SinglePositionSLAM(CoreSLAM):
    '''
    SinglePositionSLAM is an abstract class that implements CoreSLAM using a point-cloud
    with a single point (position). Implementing classes should provide the method
    
      _getNewPosition(self, start_position)
      
    to compute a new position based on searching from a starting position.
    '''

    def __init__(self, laser, map_size_pixels, map_size_meters, 
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):

        CoreSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)                    
                    
        # Initialize the position (x, y, theta)
        init_coord_mm = 500 * map_size_meters # center of map
        self.position =  pybreezyslam.Position(init_coord_mm, init_coord_mm, 0)
        
    def _updateMapAndPointcloud(self, dxy_mm, dtheta_degrees, should_update_map):
        '''
        Updates the map and point-cloud (particle cloud). Called automatically by CoreSLAM.update()
        velocities is a tuple of the form (dxy_mm, dtheta_degrees, dt_seconds).
        '''
    
        # Start at current position 
        start_pos = self.position.copy()
        
        # Add effect of velocities
        start_pos.x_mm      += dxy_mm * self._costheta()
        start_pos.y_mm      += dxy_mm * self._sintheta()
        start_pos.theta_degrees += dtheta_degrees

        # Add offset from laser
        start_pos.x_mm  += self.laser.offset_mm * self._costheta()
        start_pos.y_mm  += self.laser.offset_mm * self._sintheta()

        # Get new position from implementing class
        new_position = self._getNewPosition(start_pos)
                
        # Update the current position with this new position, adjusted by laser offset
        self.position = new_position.copy()        
        self.position.x_mm -= self.laser.offset_mm * self._costheta()
        self.position.y_mm -= self.laser.offset_mm * self._sintheta()
  
        # Update the map with this new position if indicated
        if should_update_map:
            self.map.update(self.scan_for_mapbuild, new_position, self.map_quality, self.hole_width_mm)
      
    def getpos(self):
        '''
        Returns current position as a tuple (x_mm, y_mm, theta_degrees)
        '''
        return (self.position.x_mm, self.position.y_mm, self.position.theta_degrees)
                
        
    def _costheta(self):
        
        return math.cos(self._thetaradians())
 
    def _sintheta(self):
        
        return math.sin(self._thetaradians())
        
    def _thetaradians(self):
        
        return math.radians(self.position.theta_degrees)
        
# RMHC_SLAM class ------------------------------------------------------------------------------------------------------

class RMHC_SLAM(SinglePositionSLAM):
    '''
    RMHC_SLAM implements the _getNewPosition() method of SinglePositionSLAM using Random-Mutation Hill-Climbing
    search.  Uses its own internal pseudorandom-number generator for efficiency.
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM,
                random_seed=None, sigma_xy_mm=_DEFAULT_SIGMA_XY_MM, sigma_theta_degrees=_DEFAULT_SIGMA_THETA_DEGREES, 
                max_search_iter=_DEFAULT_MAX_SEARCH_ITER):
        '''
        Creates a RMHCSlam object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        random_seed supports reproducible results; defaults to system time if unspecified
        sigma_xy_mm specifies the standard deviation in millimeters of the normal distribution of 
           the (X,Y) component of position for RMHC search
        sigma_theta_degrees specifies the standard deviation in degrees of the normal distribution of 
           the rotational component of position for RMHC search
        max_search_iter specifies the maximum number of iterations for RMHC search
        '''
    
        SinglePositionSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)
            
        if not random_seed:
            random_seed = int(time.time()) & 0xFFFF
            
        self.randomizer = pybreezyslam.Randomizer(random_seed)
        
        self.sigma_xy_mm = sigma_xy_mm
        self.sigma_theta_degrees = sigma_theta_degrees
        self.max_search_iter = max_search_iter
        
    def update(self, scans_mm, pose_change=None, scan_angles_degrees=None, should_update_map=True):

        if not pose_change:
        
            pose_change = (0, 0, 0)
    
        CoreSLAM.update(self, scans_mm, pose_change, scan_angles_degrees, should_update_map)   
    
    def _getNewPosition(self, start_position):
        '''
        Implements the _getNewPosition() method of SinglePositionSLAM. Uses Random-Mutation Hill-Climbing
        search to look for a better position based on a starting position.
        '''     
        
        # RMHC search is implemented as a C extension for efficiency
        return pybreezyslam.rmhcPositionSearch(
            start_position, 
            self.map, 
            self.scan_for_distance, 
            self.laser,
            self.sigma_xy_mm,
            self.sigma_theta_degrees,
            self.max_search_iter,
            self.randomizer)
                             
    def _random_normal(self, mu, sigma):
        
        return mu + self.randomizer.rnor() * sigma

 # Deterministic_SLAM class  ------------------------------------------------------------------------------------        

class Deterministic_SLAM(SinglePositionSLAM):
    '''
    Deterministic_SLAM implements the _getNewPosition() method of SinglePositionSLAM by simply
    copying the search-start position.
    '''
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM):
        '''
        Creates a Deterministic_Slam object suitable for updating with new Lidar and odometry data.
        laser is a Laser object representing the specifications of your Lidar unit
        map_size_pixels is the size of the square map in pixels
        map_size_meters is the size of the square map in meters
        quality from 0 through 255 determines integration speed of scan into map
        hole_width_mm determines width of obstacles (walls)
        '''
    
        SinglePositionSLAM.__init__(self, laser, map_size_pixels, map_size_meters, 
            map_quality, hole_width_mm)                    
       
    def _getNewPosition(self, start_position):
        '''
        Implements the _getNewPosition() method of SinglePositionSLAM. Returns a copy of the starting position.
        '''
        
        return start_position.copy()
