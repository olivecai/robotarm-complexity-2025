// Auto-generated. Do not edit!

// (in-package visualservo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class vs_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.iterations = null;
      this.error_vector = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('iterations')) {
        this.iterations = initObj.iterations
      }
      else {
        this.iterations = 0;
      }
      if (initObj.hasOwnProperty('error_vector')) {
        this.error_vector = initObj.error_vector
      }
      else {
        this.error_vector = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vs_info
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [iterations]
    bufferOffset = _serializer.int32(obj.iterations, buffer, bufferOffset);
    // Serialize message field [error_vector]
    bufferOffset = _arraySerializer.float32(obj.error_vector, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vs_info
    let len;
    let data = new vs_info(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [iterations]
    data.iterations = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [error_vector]
    data.error_vector = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.error_vector.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'visualservo/vs_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20b6e62368b4284b0dbbe944e7d01ad3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    int32 iterations
    float32[] error_vector
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vs_info(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.iterations !== undefined) {
      resolved.iterations = msg.iterations;
    }
    else {
      resolved.iterations = 0
    }

    if (msg.error_vector !== undefined) {
      resolved.error_vector = msg.error_vector;
    }
    else {
      resolved.error_vector = []
    }

    return resolved;
    }
};

module.exports = vs_info;
