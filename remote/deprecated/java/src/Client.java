import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.InetAddress;
import java.net.Socket;

/**
 * Connects to the Server through a socket.
 * Can send text and expects replies from server.
 */
public class Client extends Thread {

    // Define socket and input, output streams
    private InetAddress ip;                             // Host name to server
    private int port;
    private boolean connected = false;                  // Connection flag
    private DataInputStream dataInputStream;            // Input from Serial
    private DataOutputStream dataOutputStream;          // Output to Serial
    private Socket socket;                              // Connection socket
    private String header;                              // Header when sending to server

    /**
     * Client constructor.
     *
     * @param host where to connect
     * @param port communication port
     * @throws IOException When a connection can not be made.
     */
    public Client(String host, int port, String header) throws IOException {
        this.ip = InetAddress.getByName(host);
        this.port = port;
        this.header = header;

        if (ip.isReachable(1000)){
            try {
                this.socket = new Socket(this.ip, port);
            } catch (ConnectException ce) {
                ce.printStackTrace();
                System.out.println("Could not connect to server...");
            }
        }else{
            // Ip cannot be reached
            throw new IOException();
        }

        assert this.socket != null;
        this.dataInputStream = new DataInputStream(this.socket.getInputStream());
        this.dataOutputStream = new DataOutputStream(this.socket.getOutputStream());
    }

    /**
     * Connects the client to the already given ip address and port number.
     *
     * @throws IOException when connection can not be made.
     */
    public void connect() throws IOException{
        connected = ip.isReachable(1000); // Check if the host is reachable before trying to connect.
         if (connected){
             this.socket = new Socket(this.ip, this.port); // Try to connect.
         }else throw new IOException(); // Host is not reachable
        // Add the new socket to in and out streams so we can more easily interface with it.
        this.dataInputStream = new DataInputStream(this.socket.getInputStream());
        this.dataOutputStream = new DataOutputStream(this.socket.getOutputStream());
    }

    /**
     * Disconnects the client from the host.
     */
    public void disconnect(){
        try {
            this.dataInputStream.close();
            this.dataOutputStream.close();
            this.connected = false;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Send a string of data to the host. If a connection has not been made before sending, it will result
     * in a IOException being thrown.
     *
     * @param data data to be sent
     * @throws IOException when sending unsuccessfully.
     */
    public void send(String data) throws IOException {
        //String header = "POST/Controller";
        String dataToSend = header + data + "\n";
        this.dataOutputStream.writeUTF(dataToSend);
    }

    @Override
    public void run() {
        System.out.println("I ran");
    }
}