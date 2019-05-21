import lms1xx
import asvmq
import asvprotobuf.sensor_pb2
import time
import numpy as np

HOST = "10.1.24.28"
PORT = 2111
FRAME_ID = "lidar"

laser = lms1xx.LMS1xx()
pub = asvmq.Publisher(topic_name="scan", object_type=asvprotobuf.sensor_pb2.LaserScan)

while True:
    asvmq.log_info("Connecting to laser at %s" % HOST)
    laser.connect(HOST, PORT)
    if not laser.isConnected():
        asvmq.log_warn("Unable to connect, retrying.")
        time.sleep(1)
        continue

    asvmq.log_debug("Logging into the laser.")
    laser.login()
    cfg = laser.getScanCfg()
    outputRange = laser.getScanOutputRange()

    if cfg.scaningFrequency != 5000:
        laser.disconnect()
        asvmq.log_warn("Cannot get laser output range. Retrying.")
        time.sleep(1)
        continue

    asvmq.log_info("Connected to laser.")
    asvmq.log_debug("Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d" % (cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle, cfg.stopAngle));
    asvmq.log_debug("Laser output range:angleResolution %d, startAngle %d, stopAngle %d" % (outputRange.angleResolution, outputRange.startAngle, outputRange.stopAngle));

    asvmq.log_debug("Device resolution is %f degrees." % (outputRange.angleResolution/10000))
    asvmq.log_debug("Device frequency is %f Hz" % (cfg.scaningFrequency/100.0))

    angle_range = outputRange.stopAngle - outputRange.startAngle;
    num_values = angle_range/outputRange.angleResolution;
    if angle_range%outputRange.angleResolution == 0:
        num_values += 1

    dataCfg = lms1xx.scanDataCfg()
    dataCfg.outputChannel = 1
    dataCfg.remission = True
    dataCfg.resolution = 1
    dataCfg.encoder = 0
    dataCfg.position = False
    dataCfg.deviceName = False
    dataCfg.outputInterval = 1

    asvmq.log_debug("Setting scan data configuration.");
    laser.setScanDataCfg(dataCfg)

    asvmq.log_debug("Starting measurements.");
    laser.startMeas();

    asvmq.log_debug("Waiting for ready status.")

    stat = laser.queryStatus()
    time.sleep(1)
    if stat != lms1xx.status_t.ready_for_measurement:
        asvmq.log_warn("Laser not ready. Retrying initialization.")
        laser.disconnect()
        time.sleep(1)
        continue

    asvmq.log_debug("Starting device.")
    laser.startDevice();

    asvmq.log_debug("Commanding continuous measurements.")
    laser.scanContinous(1)

    while(True):
        scan_msg = asvprotobuf.sensor_pb2.LaserScan()
        #scan_msg.header.stamp = time.time()
        scan_msg.angle_increment = float(np.radians(outputRange.angleResolution/10000))
        scan_msg.angle_min = float(np.radians(outputRange.startAngle/10000)-(np.pi/2))
        scan_msg.angle_max = float(np.radians(outputRange.stopAngle/10000)-(np.pi/2))
        scan_msg.header.frame_id = FRAME_ID
        scan_msg.range_min = 0.01
        scan_msg.range_max = 20.0
        scan_msg.angle_increment = np.radians(outputRange.angleResolution/10000)
        scan_msg.angle_min = np.radians(outputRange.startAngle/10000)-np.pi/2
        scan_msg.angle_max = np.radians(outputRange.stopAngle/10000)-np.pi/2
        #asvmq.log_debug("Reading scan data.")
        t = time.time()
        data = lms1xx.scanData()
        got_input = laser.getScanData(data)
        if got_input:
            dist1 = np.trim_zeros(np.array(data.dist1), "b")*0.01
            rssi1 = np.trim_zeros(np.array(data.rssi1), "b")
            scan_msg.ranges.extend(dist1.tolist())
            scan_msg.intensities.extend(rssi1.tolist())
            #asvmq.log_debug("Publishing scan data.")
            pub.publish(scan_msg)
        else:
            asvmq.log_warn("Laser timed out on delivering scan, attempting to reinitialize.")
            break
        now = time.time()
        if(now-t<0.02):
            time.sleep(0.02-now+t)
        print("%.2f" % (1/(time.time()-t)))
    continue
laser.scanContinous(0)
laser.stopMeas()
laser.disconnect()
