import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.service.test_ros.AddTwoInts;

import com.google.common.base.Preconditions;

public class ServiceTest implements NodeMain
{
    private Node node;
    private static final String SERVICE_NAME = "/add_two_ints";
    private static final String SERVICE_TYPE = "test_ros/AddTwoInts";

    public void main(Node node)
    {
    //public void main(String[] args) {

        Preconditions.checkState(this.node == null);
        this.node = node;
        try {
            final Log log = node.getLog();

            ServiceClient<AddTwoInts.Request, AddTwoInts.Response> client =
                node.newServiceClient(SERVICE_NAME, SERVICE_TYPE);

            // TODO(damonkohler): This is a hack that we should remove once it's
            // possible to block on a connection being established.
            Thread.sleep(100);

            AddTwoInts.Request request = new AddTwoInts.Request();
            request.a = 2;
            request.b = 2;

            client.call(request, new ServiceResponseListener<AddTwoInts.Response>() {
                @Override
                public void onSuccess(AddTwoInts.Response message) {
                    log.info("I added 2 + 2: " + message.sum);
                }

                @Override
                public void onFailure(RemoteException arg0) {
                    log.info("I failed to add 2 + 2");
                }
            });

        } catch (Exception e) {
            if (node != null) {
                node.getLog().fatal(e);
            } else {
                e.printStackTrace();
            }
        }
    }

    public void shutdown() {
        node.shutdown();
        node = null;
    }

    @Override
    public void onShutdown(Node arg0) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onShutdownComplete(Node arg0) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onStart(Node arg0) {

    }

}