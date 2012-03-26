
#include <cstdio>
#include <string>
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/module.h"
#include "ppapi/cpp/var.h"

#include <json/json.h>
#include <btBulletDynamicsCommon.h>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <sstream>

#include <map>

class PhysWorkerInstance : public pp::Instance {
 public:
  explicit PhysWorkerInstance(PP_Instance instance) : pp::Instance(instance) {
      world = 0;
      last_simulation_time = 0.0;
  }
  virtual ~PhysWorkerInstance() {}

    std::ostringstream buff;
    btDiscreteDynamicsWorld* world;
    double fixedTimeStep;
    double last_simulation_time;
    std::map<int,btRigidBody*> objects;
    std::map<btRigidBody*,int> objects_ptr;

    void init(Json::Value params) {
        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
        btBroadphaseInterface* broadphase;
        if (params.isObject() && !!params["broadphase"] && params["broadphase"].isObject() && !!params["broadphase"]["type"] && params["broadphase"]["type"].isString()) {
            std::string type = params["broadphase"]["type"].asString();
            Json::Value bb = params["broadphase"];
            if (type == "sweepprune") broadphase = new btAxisSweep3(objectVector(bb, "aabbmin"), objectVector(bb, "aabbmax"));
            else broadphase = new btDbvtBroadphase();
        }
        else broadphase = new btDbvtBroadphase();
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
        this->world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
        this->fixedTimeStep = 1.0/60.0;
        if (params.isObject() && !!params["fixedTimeStep"] && params["fixedTimeStep"].isNumeric()) this->fixedTimeStep = params["fixedTimeStep"].asDouble();
        // PostMessage(pp::Var("Initialization success"));
        
        objects.clear();
        objects_ptr.clear();
        
    }
    
    btVector3 makeVector(Json::Value obj) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (obj.isObject()) {
            if (!!obj["x"] && obj["x"].isNumeric()) x = obj["x"].asDouble(); 
            if (!!obj["y"] && obj["y"].isNumeric()) y = obj["y"].asDouble(); 
            if (!!obj["z"] && obj["z"].isNumeric()) z = obj["z"].asDouble(); 
        }
        return btVector3(x,y,z);
    }

    btVector3 makeVector(Json::Value obj, std::string x_str, std::string y_str, std::string z_str) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (obj.isObject()) {
            if (!!obj[x_str] && obj[x_str].isNumeric()) x = obj[x_str].asDouble(); 
            if (!!obj[y_str] && obj[y_str].isNumeric()) y = obj[y_str].asDouble(); 
            if (!!obj[z_str] && obj[z_str].isNumeric()) z = obj[z_str].asDouble(); 
        }
        return btVector3(x,y,z);
    }

    btQuaternion makeQuaternion(Json::Value obj) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
        if (obj.isObject()) {
            if (!!obj["x"] && obj["x"].isNumeric()) x = obj["x"].asDouble(); 
            if (!!obj["y"] && obj["y"].isNumeric()) y = obj["y"].asDouble(); 
            if (!!obj["z"] && obj["z"].isNumeric()) z = obj["z"].asDouble(); 
            if (!!obj["w"] && obj["w"].isNumeric()) w = obj["w"].asDouble(); 
        }
        return btQuaternion(x,y,z,w);
    }

    double objectNumber(Json::Value obj, std::string field) {
        if (obj.isObject() && !!obj[field] && obj[field].isNumeric()) return obj[field].asDouble();
        else return 0.0;
    }

    btVector3 objectVector(Json::Value obj, std::string field) {
        if (obj.isObject() && !!obj[field]) return makeVector(obj[field]);
        else return btVector3(0.0, 0.0, 0.0);
    }

    void setGravity(Json::Value gravity) {
        if (!world) return;
        world->setGravity(makeVector(gravity));
    }

    void addObject(Json::Value description) {
        btTransform transform;
        transform.setIdentity();
        btVector3 localInertia(0.0, 0.0, 0.0);
        if (!description.isObject() || !description["type"] || !description["type"].isString()) return;
        std::string type = description["type"].asString();
        btCollisionShape *shape;
        if (type == "plane") {
            shape = new btStaticPlaneShape(makeVector(description["normal"]), 0.0);
        }
        else if (type == "box") {
            shape = new btBoxShape(btVector3(objectNumber(description, "width") / 2.0, objectNumber(description, "height") / 2.0, objectNumber(description, "depth") / 2.0));
        }
        else if (type == "cylinder") {
            shape = new btCylinderShape(btVector3(objectNumber(description, "width") / 2.0, objectNumber(description, "height") / 2.0, objectNumber(description, "depth") / 2.0));
        }
        else if (type == "sphere") {
            shape = new btSphereShape(objectNumber(description, "radius"));
        }
        else if (type == "cone") {
            shape = new btConeShape(objectNumber(description, "radius"), objectNumber(description, "height"));
        }
        else return;
        // custom, heightmap
        
        shape->calculateLocalInertia(objectNumber(description, "mass"), localInertia);
        btDefaultMotionState *motionState = new btDefaultMotionState(transform); // #TODO: btDefaultMotionState supports center of mass offset as second argument - implement
	    btRigidBody::btRigidBodyConstructionInfo rbInfo(objectNumber(description,"mass"), motionState, shape, localInertia);
        
        if (description["friction"].isNumeric()) rbInfo.m_friction = objectNumber(description,"friction");
        if (description["restitution"].isNumeric()) rbInfo.m_restitution = objectNumber(description,"restitution");
        
        btRigidBody* body = new btRigidBody(rbInfo);
        
        if (description["collision_flags"].isInt()) {
            body->setCollisionFlags(description["collision_flags"].asInt());
        }

        world->addRigidBody(body);

        if (description["id"].isInt()) {
            objects[description["id"].asInt()] = body;
            objects_ptr[body] = description["id"].asInt();
            // PostMessage(pp::Var("Adding object worked!"));
        }

    }

    void updateTransform(Json::Value details) {
        
        if (details["id"].isInt() && objects[details["id"].asInt()]) {
            btRigidBody *object = objects[details["id"].asInt()];
            btTransform transform;
            object->getMotionState()->getWorldTransform(transform);
            if (details["pos"].isObject()) {
                transform.setOrigin(makeVector(details["pos"]));
            }
            if (details["quat"].isObject()) {
                transform.setRotation(makeQuaternion(details["quat"]));
            }
            object->setWorldTransform(transform);
            object->activate();
        }
        
    }

    void removeObject(Json::Value details) {        
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            int id = details["id"].asInt();
            btRigidBody *object = objects[id];
            objects.erase(id);
            objects_ptr.erase(object);
            world->removeRigidBody(object);
            // PostMessage(pp::Var("Remove success!"));
        }
        
    }

    void updateMass(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setMassProps(objectNumber(details, "mass"), btVector3(0.0, 0.0, 0.0));
            world->removeRigidBody(object);
            world->addRigidBody(object);
    	    object->activate();
        }
        
    }

    void applyCentralImpulse(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->applyCentralImpulse(makeVector(details));
            object->activate();
        }
    }

    void applyImpulse(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->applyImpulse(makeVector(details), makeVector(details, "impulse_x", "impulse_y", "impulse_z"));
            object->activate();
        }
    }

    void setAngularVelocity(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setAngularVelocity(makeVector(details));
            object->activate();
        }
    }

    void setAngularFactor(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setAngularFactor(makeVector(details));
            object->activate();
        }
    }

    void setLinearVelocity(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setLinearVelocity(makeVector(details));
        }
    }

    void setLinearFactor(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setLinearFactor(makeVector(details));
        }
    }

    void setCcdMotionThreshold(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setCcdMotionThreshold(objectNumber(details, "threshold"));
        }
    }

    void setCcdSweptSphereRadius(Json::Value details) {
        if (details["id"].isInt() && objects.count(details["id"].asInt()) > 0) {
            btRigidBody *object = objects[details["id"].asInt()];
            object->setCcdSweptSphereRadius(objectNumber(details, "radius"));
        }
    }

    double getTime() {
        timeval a;
        gettimeofday(&a, NULL);
        return a.tv_sec + a.tv_usec / 1000000.0;
    }
    
    void simulate(Json::Value params) {
        if (world) {
            double now = getTime();
            double timeStep;
            if (params.isObject() && !!params["timeStep"]) {
                timeStep = objectNumber(params, "timeStep");
            }
            else {
                if (last_simulation_time > 0) {
                    timeStep = now - last_simulation_time;
                }
                else {
                    timeStep = fixedTimeStep;
                }
            }
            
            int maxSubSteps;
            if (params.isObject() && !!params["maxSubSteps"] && params["maxSubSteps"].isInt()) {
                maxSubSteps = params["maxSubSteps"].asInt();
            }
            else {
                maxSubSteps = int(timeStep / fixedTimeStep) + 1;
            }
            
            world->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
            
            double start = getTime();
            
            reportWorld();

            // PostMessage(pp::Var(Json::FastWriter().write(getTime() - start)));
            
            last_simulation_time = now;
            // Json::FastWriter writer;
            // Json::Value val(timeStep);
            // PostMessage(pp::Var("asd"));
        }
    }
/*
            buff << "[";
            buff << (*i).first << ",";
            buff << origin.x() << ",";
            buff << origin.y() << ",";
            buff << origin.z() << ",";
            buff << rotation.x() << ",";
            buff << rotation.y() << ",";
            buff << rotation.z() << ",";
            buff << rotation.w() << "]";
            buff << "[";
            buff << "0.0,";
            buff << "0.0,";
            buff << "0.0,";
            buff << "0.0,";
            buff << "0.0,";
            buff << "0.0,";
            buff << "0.0,";
            buff << "0.0]";
*/
    void reportWorld() {
        /*
        buff.str("");
        buff << "{\"cmd\":\"update\", \"params\":{\"objects\":[";
        int idx = 0;
        for (std::map<int,btRigidBody*>::iterator i = objects.begin(); i != objects.end(); ++i) {
            if (idx > 0) buff << ",";
            idx++;
            btTransform transform = (*i).second->getCenterOfMassTransform();
            btVector3 origin = transform.getOrigin();
            btQuaternion rotation = transform.getRotation();
            Json::Value obj(Json::objectValue);
            buff << "{";
            buff << "\"id\":" << (*i).first << ",";
            buff << "\"pos_x\":" << origin.x() << ",";
            buff << "\"pos_y\":" << origin.y() << ",";
            buff << "\"pos_z\":" << origin.z() << ",";
            buff << "\"quat_x\":" << rotation.x() << ",";
            buff << "\"quat_y\":" << rotation.y() << ",";
            buff << "\"quat_z\":" << rotation.z() << ",";
            buff << "\"quat_w\":" << rotation.w() << "}";
        }
        buff << "]}}";
        PostMessage(pp::Var(buff.str()));
        */
        Json::Value report(Json::arrayValue);
        int idx = 0;
        for (std::map<int,btRigidBody*>::iterator i = objects.begin(); i != objects.end(); ++i) {
            btTransform transform = (*i).second->getCenterOfMassTransform();
            btVector3 origin = transform.getOrigin();
            btQuaternion rotation = transform.getRotation();
            Json::Value obj(Json::objectValue);
            btVector3 vector1 = (*i).second->getLinearVelocity();
            btVector3 vector2 = (*i).second->getAngularVelocity();
            obj["id"] = (*i).first;
            obj["pos_x"] = origin.x();
            obj["pos_y"] = origin.y();
            obj["pos_z"] = origin.z();
            obj["quat_x"] = rotation.x();
            obj["quat_y"] = rotation.y();
            obj["quat_z"] = rotation.z();
            obj["quat_w"] = rotation.w();
            obj["linear_x"] = vector1.x();
            obj["linear_y"] = vector1.y();
            obj["linear_z"] = vector1.z();
            obj["angular_x"] = vector2.x();
            obj["angular_y"] = vector2.y();
            obj["angular_z"] = vector2.z();
            report[idx++] = obj;
        }
        // Add collisions
        Json::Value collisions0(Json::arrayValue);
        Json::Value collisions1(Json::arrayValue);
        btDispatcher* dp = world->getDispatcher();
        int num = dp->getNumManifolds();
        for (int i = 0; i < num; i++) {
            btPersistentManifold *manifold = dp->getManifoldByIndexInternal(i);
            int num_contacts = manifold->getNumContacts();
            for (int j = 0; j < num_contacts; j++) {
                // btManifoldPoint& pt = manifold->getContactPoint(j);
                // report[objects_ptr[(btRigidBody*)manifold->getBody0()]]["collisions"]
                collisions0.append(objects_ptr[(btRigidBody*)manifold->getBody0()]);
                collisions1.append(objects_ptr[(btRigidBody*)manifold->getBody1()]);
            }
        }
        
        // Create return value
        Json::Value res(Json::objectValue);
        res["cmd"] = "update";
        res["params"] = Json::Value(Json::objectValue);
        res["params"]["objects"] = report;
        res["params"]["collisions0"] = collisions0;
        res["params"]["collisions1"] = collisions1;
        PostMessage(pp::Var(json_writer.write(res)));
        // pp::Var(writer.write(res));
    }

    Json::FastWriter json_writer;

    virtual void HandleMessage(const pp::Var& var_message) {
    	  
    	if (!var_message.is_string()) {
            PostMessage(pp::Var(var_message.DebugString()));
            return;
    	}
    
    	std::string message = var_message.AsString();
    	pp::Var var_reply;
        Json::Reader reader = Json::Reader();
        Json::Value val;
        if (reader.parse(message, val)) {
            if (!val.isObject() || !val["cmd"] || !val["cmd"].isString()) {
                PostMessage(pp::Var("Invalid command!"));
                return;
            }
            std::string cmd = val["cmd"].asString();
            if (cmd == "init") {
                this->init(val["params"]);
            }
            else if (cmd == "setGravity") {
                if (val["params"].isObject()) this->setGravity(val["params"]);
            }
            else if (cmd == "addObject") {
                if (val["params"].isObject()) this->addObject(val["params"]);
            }
            else if (cmd == "removeObject") {
                if (val["params"].isObject()) this->removeObject(val["params"]);
            }
            else if (cmd == "updateTransform") {
                if (val["params"].isObject()) this->updateTransform(val["params"]);
            }
            else if (cmd == "updateMass") {
                if (val["params"].isObject()) this->updateMass(val["params"]);
            }
            else if (cmd == "applyCentralImpulse") {
                if (val["params"].isObject()) this->applyCentralImpulse(val["params"]);
            }
            else if (cmd == "applyImpulse") {
                if (val["params"].isObject()) this->applyImpulse(val["params"]);
            }
            else if (cmd == "setAngularVelocity") {
                if (val["params"].isObject()) this->setAngularVelocity(val["params"]);
            }
            else if (cmd == "setAngularFactor") {
                if (val["params"].isObject()) this->setAngularFactor(val["params"]);
            }
            else if (cmd == "setLinearVelocity") {
                if (val["params"].isObject()) this->setLinearVelocity(val["params"]);
            }
            else if (cmd == "setLinearFactor") {
                if (val["params"].isObject()) this->setLinearFactor(val["params"]);
            }
            else if (cmd == "setCcdMotionThreshold") {
                if (val["params"].isObject()) this->setCcdMotionThreshold(val["params"]);
            }
            else if (cmd == "setCcdSweptSphereRadius") {
                if (val["params"].isObject()) this->setCcdSweptSphereRadius(val["params"]);
            }
            else if (cmd == "simulate") {
                this->simulate(val["params"]);
            }
            else PostMessage(pp::Var("Unknown command"));
        }
    }
   
};

class PhysWorkerModule : public pp::Module {
 public:
  PhysWorkerModule() : pp::Module() {}
  virtual ~PhysWorkerModule() {}

  virtual pp::Instance* CreateInstance(PP_Instance instance) {
    return new PhysWorkerInstance(instance);
  }
};

namespace pp {
Module* CreateModule() {
  return new PhysWorkerModule();
}
}  // namespace pp
