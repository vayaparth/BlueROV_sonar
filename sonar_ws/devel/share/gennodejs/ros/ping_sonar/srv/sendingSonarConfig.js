// Auto-generated. Do not edit!

// (in-package ping_sonar.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class sendingSonarConfigRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stepSize = null;
      this.range = null;
    }
    else {
      if (initObj.hasOwnProperty('stepSize')) {
        this.stepSize = initObj.stepSize
      }
      else {
        this.stepSize = 0;
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sendingSonarConfigRequest
    // Serialize message field [stepSize]
    bufferOffset = _serializer.int64(obj.stepSize, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.int64(obj.range, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sendingSonarConfigRequest
    let len;
    let data = new sendingSonarConfigRequest(null);
    // Deserialize message field [stepSize]
    data.stepSize = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ping_sonar/sendingSonarConfigRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8668035c539b1dc98b4832c66a9cd827';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 stepSize
    int64 range
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sendingSonarConfigRequest(null);
    if (msg.stepSize !== undefined) {
      resolved.stepSize = msg.stepSize;
    }
    else {
      resolved.stepSize = 0
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0
    }

    return resolved;
    }
};

class sendingSonarConfigResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.worked = null;
    }
    else {
      if (initObj.hasOwnProperty('worked')) {
        this.worked = initObj.worked
      }
      else {
        this.worked = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sendingSonarConfigResponse
    // Serialize message field [worked]
    bufferOffset = _serializer.bool(obj.worked, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sendingSonarConfigResponse
    let len;
    let data = new sendingSonarConfigResponse(null);
    // Deserialize message field [worked]
    data.worked = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ping_sonar/sendingSonarConfigResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9488d8ea76faf703564dab31aa285e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool worked
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sendingSonarConfigResponse(null);
    if (msg.worked !== undefined) {
      resolved.worked = msg.worked;
    }
    else {
      resolved.worked = false
    }

    return resolved;
    }
};

module.exports = {
  Request: sendingSonarConfigRequest,
  Response: sendingSonarConfigResponse,
  md5sum() { return '6cb2431d101e8b181456b4863d041954'; },
  datatype() { return 'ping_sonar/sendingSonarConfig'; }
};
