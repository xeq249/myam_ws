// Auto-generated. Do not edit!

// (in-package real_test.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AddEyeToHandSampleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base_x = null;
      this.base_y = null;
      this.base_z = null;
      this.optical_x = null;
      this.optical_y = null;
      this.optical_z = null;
    }
    else {
      if (initObj.hasOwnProperty('base_x')) {
        this.base_x = initObj.base_x
      }
      else {
        this.base_x = 0.0;
      }
      if (initObj.hasOwnProperty('base_y')) {
        this.base_y = initObj.base_y
      }
      else {
        this.base_y = 0.0;
      }
      if (initObj.hasOwnProperty('base_z')) {
        this.base_z = initObj.base_z
      }
      else {
        this.base_z = 0.0;
      }
      if (initObj.hasOwnProperty('optical_x')) {
        this.optical_x = initObj.optical_x
      }
      else {
        this.optical_x = 0.0;
      }
      if (initObj.hasOwnProperty('optical_y')) {
        this.optical_y = initObj.optical_y
      }
      else {
        this.optical_y = 0.0;
      }
      if (initObj.hasOwnProperty('optical_z')) {
        this.optical_z = initObj.optical_z
      }
      else {
        this.optical_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddEyeToHandSampleRequest
    // Serialize message field [base_x]
    bufferOffset = _serializer.float32(obj.base_x, buffer, bufferOffset);
    // Serialize message field [base_y]
    bufferOffset = _serializer.float32(obj.base_y, buffer, bufferOffset);
    // Serialize message field [base_z]
    bufferOffset = _serializer.float32(obj.base_z, buffer, bufferOffset);
    // Serialize message field [optical_x]
    bufferOffset = _serializer.float32(obj.optical_x, buffer, bufferOffset);
    // Serialize message field [optical_y]
    bufferOffset = _serializer.float32(obj.optical_y, buffer, bufferOffset);
    // Serialize message field [optical_z]
    bufferOffset = _serializer.float32(obj.optical_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddEyeToHandSampleRequest
    let len;
    let data = new AddEyeToHandSampleRequest(null);
    // Deserialize message field [base_x]
    data.base_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [base_y]
    data.base_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [base_z]
    data.base_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [optical_x]
    data.optical_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [optical_y]
    data.optical_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [optical_z]
    data.optical_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'real_test/AddEyeToHandSampleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '855ed419012aa2e204e450eb6966a5a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 手动增加一对 3D 对应点：同一点在基座系与相机光学系下的坐标（米）
    float32 base_x
    float32 base_y
    float32 base_z
    float32 optical_x
    float32 optical_y
    float32 optical_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddEyeToHandSampleRequest(null);
    if (msg.base_x !== undefined) {
      resolved.base_x = msg.base_x;
    }
    else {
      resolved.base_x = 0.0
    }

    if (msg.base_y !== undefined) {
      resolved.base_y = msg.base_y;
    }
    else {
      resolved.base_y = 0.0
    }

    if (msg.base_z !== undefined) {
      resolved.base_z = msg.base_z;
    }
    else {
      resolved.base_z = 0.0
    }

    if (msg.optical_x !== undefined) {
      resolved.optical_x = msg.optical_x;
    }
    else {
      resolved.optical_x = 0.0
    }

    if (msg.optical_y !== undefined) {
      resolved.optical_y = msg.optical_y;
    }
    else {
      resolved.optical_y = 0.0
    }

    if (msg.optical_z !== undefined) {
      resolved.optical_z = msg.optical_z;
    }
    else {
      resolved.optical_z = 0.0
    }

    return resolved;
    }
};

class AddEyeToHandSampleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.count = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddEyeToHandSampleResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.int32(obj.count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddEyeToHandSampleResponse
    let len;
    let data = new AddEyeToHandSampleResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'real_test/AddEyeToHandSampleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c80d4d2084ba02c680df535380f6464';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    int32 count
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddEyeToHandSampleResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: AddEyeToHandSampleRequest,
  Response: AddEyeToHandSampleResponse,
  md5sum() { return 'b1d912318536383476f3a61d6d9c6b77'; },
  datatype() { return 'real_test/AddEyeToHandSample'; }
};
