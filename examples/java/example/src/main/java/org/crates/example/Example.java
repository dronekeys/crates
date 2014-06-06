/*
 * Copyright (C) 2014 Andrew Symington.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.crates.example;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.exception.RosRuntimeException;

// Standard ROS messages
import org.ros.message.*;

// CRATES messages
import org.crates.*;


public class Example extends AbstractNodeMain
{

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/listener");
  }

  @Override
  public void onStart(final ConnectedNode nh)
  {
    // Attach to the console
    final Log log = nh.getLog();

    // Declare all the services in advance
    final ServiceClient<sim.PauseRequest,  sim.PauseResponse>  srvPause;
    final ServiceClient<sim.InsertRequest, sim.InsertResponse> srvInsert;
    final ServiceClient<sim.ResumeRequest, sim.ResumeResponse> srvResume;

    // Try and connect to the services
    try
    {
      srvPause  = nh.newServiceClient("/simulator/Pause",  sim.Pause._TYPE);
      srvInsert = nh.newServiceClient("/simulator/Insert", sim.Insert._TYPE);
      srvResume = nh.newServiceClient("/simulator/Resume", sim.Resume._TYPE);
    }
    catch (ServiceNotFoundException e)
    {
      throw new RosRuntimeException(e);
    }

    // Create a Pause request and send it to the simulator
    sim.PauseRequest reqPause = nh.getTopicMessageFactory().newFromType(sim.PauseRequest._TYPE);     
    srvPause.call(reqPause, new ServiceResponseListener<sim.PauseResponse>() {
      
      // On successfully pausing...
      @Override
      public void onSuccess(sim.PauseResponse message)
      {
        // Create a request and call the service
        sim.InsertRequest reqInsert = nh.getTopicMessageFactory().newFromType(sim.InsertRequest._TYPE);
        reqInsert.setModelName("UAV0");
        reqInsert.setModelType("model://hummingbird");
        srvInsert.call(reqInsert, new ServiceResponseListener<sim.InsertResponse>(){
          
          // On successfully inserting...
          @Override
          public void onSuccess(sim.InsertResponse message)
          {
            // Create a request and call the service
            sim.ResumeRequest reqResume = nh.getTopicMessageFactory().newFromType(sim.ResumeRequest._TYPE);
            srvResume.call(reqResume, new ServiceResponseListener<sim.ResumeResponse>() {
              
              // On successfully inserting...
              @Override
              public void onSuccess(sim.ResumeResponse message)
              {
                // Subscribe to estimated state of the recently-created UAV
                Subscriber<hal_quadrotor.State> subscriber = nh.newSubscriber("/hal/UAV0/Estimate", hal_quadrotor.State._TYPE);
                subscriber.addMessageListener(new MessageListener<hal_quadrotor.State>() {
                  @Override
                  public void onNewMessage(hal_quadrotor.State message) {
                    log.info("Quadrotor position: [" + message.getX() + "," + message.getY() + "," + message.getZ() + "]");
                  }
                });
              }

              // If there was an issue inserting...
              @Override
              public void onFailure(RemoteException arg0)
              {
                throw new RosRuntimeException("Could not resume the simulator");
              }
            });

          }

          // If there was an issue inserting...
          @Override
          public void onFailure(RemoteException arg0)
          {
            throw new RosRuntimeException("Could not insert new model into the simulator");
          }
        });

      }

      // If there was an issue pausing...
      @Override
      public void onFailure(RemoteException arg0)
      {
        throw new RosRuntimeException("Could not pause the simulator");
      }
    });
  }
}

