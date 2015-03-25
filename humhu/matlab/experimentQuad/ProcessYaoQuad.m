%% Dataset processing for Yao quadrotor dataset
bagRoot = '~/Software/CovarianceRegression/data/yao_quad/bagfiles/';
bagFiles = {    'delta_141216_B', ...
    'hotel_141014_E', ...
    'hotel_141014_H', ...
    'hotel_141030_B', ...
    'hotel_141103_G', ...
    'hotel_141116_A', ...
    'hotel_141201_A' };
numBags = numel(bagFiles);

quadNames = { 'delta', 'hotel', 'hotel', 'hotel', 'hotel', 'hotel', 'hotel' };

matRoot = '~/Software/CovarianceRegression/data/yao_quad/mat/';

%% Create mat files for each bag

for i = 1:numBags

    bagName = bagFiles{i};
    bagPath = [ bagRoot, bagName, '.bag' ];
    bag = ros.Bag.load( bagPath );

    fprintf( ['Processing ', bagName, '...\n'] );

    numTopics = numel( bag.topics );
    varnames = {};
    for j = 1:numTopics

        fsm_eacc = bag.readAll( ['/', quadNames{i}, '/controller/fsm/eacc' ] );
        fsm_eacc = [fsm_eacc{:}];
        fsm_epos = bag.readAll( ['/', quadNames{i}, '/controller/fsm/epos' ] );
        fsm_epos = [fsm_epos{:}];
        fsm_evel = bag.readAll( ['/', quadNames{i}, '/controller/fsm/evel' ] );
        fsm_evel = [fsm_evel{:}];
        fsm_ttrim = bag.readAll( ['/', quadNames{i}, '/controller/fsm/transient_trim' ] );
        fsm_ttrim = [fsm_ttrim{:}];
        cpu = bag.readAll( ['/', quadNames{i}, '/cpu_monitor/cpu_status' ] );
        cpu = [cpu{:}];
        imu = bag.readAll( ['/', quadNames{i}, '/imu' ] );
        imu = [imu{:}];
        vicon = bag.readAll( ['/', quadNames{i}, '/odom' ] );
        vicon = [vicon{:}];
        cmd = bag.readAll( ['/', quadNames{i}, '/pd_cmd' ] );
        cmd = [cmd{:}];
        scan = bag.readAll( ['/', quadNames{i}, '/scan' ] );
        scan = [scan{:}];
        slamAlt = bag.readAll( ['/', quadNames{i}, '/slam/altitude' ] );
        slamAlt = [slamAlt{:}];
        icpInc = bag.readAll( ['/', quadNames{i}, '/slam/icp/incremental_pose' ] );
        icpInc = [icpInc{:}];
        slamFilteredPose = bag.readAll( ['/', quadNames{i}, '/slam/localization/lp_pose' ] );
        slamFilteredPose = [slamFilteredPose{:}];
        slamPose = bag.readAll( ['/', quadNames{i}, '/slam/localization/pose' ] );
        slamPose = [slamPose{:}];
        cor_a2d = bag.readAll( ['/', quadNames{i}, '/slam/ukf/cor_a2d' ] );
        cor_a2d = [cor_a2d{:}];
        cor_alt = bag.readAll( ['/', quadNames{i}, '/slam/ukf/cor_alt' ] );
        cor_alt = [cor_alt{:}];
        cor_r2d = bag.readAll( ['/', quadNames{i}, '/slam/ukf/cor_r2d' ] );
        cor_r2d = [cor_r2d{:}];
        odom = bag.readAll( ['/', quadNames{i}, '/slam/ukf/odom' ] );
        odom = [odom{:}];
        voltage = bag.readAll( ['/', quadNames{i}, '/voltage' ] );
        voltage = [voltage{:}];

        matName = [matRoot, bagName, '.mat'];
        vars = {'fsm_eacc', 'fsm_epos', 'fsm_evel', 'fsm_ttrim', 'cpu', ...
            'imu', 'vicon', 'cmd', 'scan', 'slamAlt', 'icpInc', 'slamFilteredPose', ...
            'slamPose', 'cor_a2d', 'cor_alt', 'cor_r2d', 'odom', 'voltage' };
        save( matName, vars{:} );

    end

end

%% Pull out relevant fields
for i = 1:numBags
    
    matName = [matRoot, bagFiles{i}, '.mat'];
    load( matName );
    
    fprintf( ['Processing ', bagFiles{i}, '\n'] );
    
    % Store laser scans
    scanHeaders = [scan.header];
    scanStamps = [scanHeaders.stamp];
    scanTimes = [scanStamps.time];
    scans = LaserScan2D( scanTimes, [[scan.angle_min]; [scan.angle_max]], [scan.ranges] );
    
    % Store complementary filter orientations
    nCf = numel( imu );
    cfHeaders = [imu.header];
    cfStamps = [cfHeaders.stamp];
    cfTimes = [cfStamps.time];
    cfQuats = [imu.orientation];
    cfEuler = quat2euler(cfQuats);
    cf = struct( 'timestamp', num2cell( cfTimes, 1 ), ...
        'quaternion', num2cell( cfQuats, 1 ), ...
        'euler', num2cell( cfEuler, 1 ) );
    
    % Store IMU measurements
    nIMU = numel( imu );
    imuTimes = cfTimes;
    imuAngularVelocities = [imu.angular_velocity];
    imuLinearAcceleration = [imu.linear_acceleration];
    imuMisc = [imu.orientation_covariance];
    imuMagnetometer = imuMisc(1:3,:);
    imuTemperature = imuMisc(4,:);
    imuPressure = imuMisc(5:6,:);
    imu = struct( 'timestamp', num2cell( imuTimes, 1 ), ...
        'angularVelocity', num2cell( imuAngularVelocities, 1 ), ...
        'linearAcceleration', num2cell( imuLinearAcceleration, 1 ), ...
        'magnetometer', num2cell( imuMagnetometer, 1 ), ...
        'temperature', num2cell( imuTemperature, 1 ), ...
        'pressure', num2cell( imuPressure, 1 ) );
    
    % Store ICP measurements
    if ~isempty(cor_r2d)
        nICP = numel( cor_r2d );
        icpHeaders = [cor_r2d.header];
        icpStamps = [icpHeaders.stamp];
        icpTimes = [icpStamps.time];
        icpEstimated = [cor_r2d.zest];
        icpActual = [cor_r2d.z];
        icpS = reshape( [cor_r2d.S], [3, 3, nICP] );
        icpR = reshape( [cor_r2d.R], [3, 3, nICP] );
        icpD2 = [cor_r2d.d2];
        icp = struct( 'timestamp', num2cell( icpTimes, 1 ), ...
            'estimated', num2cell( icpEstimated, 1 ), ...
            'observed', num2cell( icpActual, 1 ), ...
            'mappedPoseCovariance', squeeze(num2cell( icpS, [1,2] ))', ...
            'measurementCovariance', squeeze(num2cell( icpR, [1,2] ))', ...
            'd2', num2cell( icpD2, 1 ) );
    else
        icp = [];
    end
    
    % Store histogram global measurements
    if ~isempty( cor_a2d )
        nSLAM = numel( cor_a2d );
        slamHeaders = [cor_a2d.header];
        slamStamps = [slamHeaders.stamp];
        slamTimes = [slamStamps.time];
        slamEstimated = [cor_a2d.zest];
        slamActual = [cor_a2d.z];
        slamS = reshape( [cor_a2d.S], [3, 3, nSLAM] );
        slamR = reshape( [cor_a2d.R], [3, 3, nSLAM] );
        slamD2 = [cor_a2d.d2];
        slam = struct( 'timestamp', num2cell( slamTimes, 1 ), ...
            'estimated', num2cell( slamEstimated, 1 ), ...
            'observed', num2cell( slamActual, 1 ), ...
            'mappedPoseCovariance', squeeze(num2cell( slamS, [1,2] ))', ...
            'measurementCovariance', squeeze(num2cell( slamR, [1,2] ))', ...
            'd2', num2cell( slamD2, 1 ) );
    else
        slam = [];
    end
    
    % Store altitude measurements
    if ~isempty( cor_alt )
        nAlt = numel( cor_alt );
        altHeaders = [cor_alt.header];
        altStamps = [altHeaders.stamp];
        altTimes = [altStamps.time];
        altEstimated = [cor_alt.zest];
        altActual = [cor_alt.z];
        altS = [cor_alt.S];
        altR = [cor_alt.R];
        altD2 = [cor_alt.d2];
        alt = struct( 'timestamp', num2cell( altTimes, 1 ), ...
            'estimated', num2cell( altEstimated, 1 ), ...
            'observed', num2cell( altActual, 1 ), ...
            'mappedPoseCovariance', num2cell( altS, 1 ), ...
            'measurementCovariance', num2cell( altR, 1 ), ...
            'd2', num2cell( altD2, 1 ) );
    else
        alt = [];
    end
    
    dataName = [matRoot, bagFiles{i}, '_data.mat' ];
    save( dataName, 'scans', 'cf', 'imu', 'icp', 'slam', 'alt', 'cmd', 'vicon' );
    
end
