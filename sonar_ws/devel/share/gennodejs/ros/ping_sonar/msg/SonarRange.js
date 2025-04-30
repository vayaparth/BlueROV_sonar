// Auto-generated. Do not edit!

// (in-package ping_sonar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SonarRange {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sonar_range = null;
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('sonar_range')) {
        this.sonar_range = initObj.sonar_range
      }
      else {
        this.sonar_range = [];
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SonarRange
    // Serialize message field [sonar_range]
    bufferOffset = _arraySerializer.float64(obj.sonar_range, buffer, bufferOffset, null);
    // Serialize message field [angle]
    bufferOffset = _arraySerializer.float64(obj.angle, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SonarRange
    let len;
    let data = new SonarRange(null);
    // Deserialize message field [sonar_range]
    data.sonar_range = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [angle]
    data.angle = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.sonar_range.length;
    length += 8 * object.angle.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ping_sonar/SonarRange';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '635d4ca17ad4fed19b4f5bcdfb57f952';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] sonar_range # len: 360/step_size, 0->front
    float64[] angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SonarRange(null);
    if (msg.sonar_range !== undefined) {
      resolved.sonar_range = msg.sonar_range;
    }
    else {
      resolved.sonar_range = []
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = []
    }

    return resolved;
    }
};

module.exports = SonarRange;
