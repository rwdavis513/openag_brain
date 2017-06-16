import os, time
import requests
import subprocess
import json
import pdb
import pexpect
import random
import uuid
from settings import SERVER_URL, SSH_FOLDER, user_account

login_info = {
        "password": user_account['password'],
        "email": user_account['email']
        }

# TODO: Add Tests for each function
# TODO: Add check to account for RSA Fingerprint text
# TODO: Modify UUID to a hash of the mac address or something unique to the pfc
# TOOD: Modify the functions to be pure functions so they don't modify the session variable


def get_tunnel_info():
    """ 
    Checks if the tunnel info already exists, if not then it requests new information
    """
    prior_tunnel_info = check_for_tunnel_info()
    if prior_tunnel_info:
        return prior_tunnel_info
    else:
        session = requests.Session()
        register_login(session)
        new_tunnel_info = get_new_tunnel_info(session)
        return new_tunnel_info


def register_login(session, register_user=False):
    """
    Logs the User into the cloud software
    """
    REGISTER_URL = "/auth"
    if register_user:
        # PUT Registers a NEW user
        response = session.put(SERVER_URL + REGISTER_URL, json=login_info)
        if 200 <= response.status_code <= 210:
            return True
        else:
            print(response.text)
            pdb.set_trace()
            raise Exception("Failed to Register")
    else:
        # POST Logs the User in
        response = session.post(SERVER_URL + REGISTER_URL, json=login_info)
        if 200 <= response.status_code <= 210:
            return True
        else:
            print(response.text)
            pdb.set_trace()
            raise Exception("Failed to Login")


def get_new_tunnel_info(session):
    TUNNEL_URL = '/tunnel'
    response = session.put(SERVER_URL + TUNNEL_URL)
    if response.status_code == 201:
        return response.json()
    else:
        print(response.text)
        pdb.set_trace()
        raise Exception("Error: Resceived error code {}".format(response.status_code))


def save_tunnel_info(tunnel_info, file_name='tunnel_info.json'):
    file_name = os.path.join(SSH_FOLDER, file_name)
    if type(tunnel_info) != dict:
        raise Exception("Error: Wrong data type")
    with open(file_name, 'w') as f:
        json.dump(tunnel_info, f)


def check_for_tunnel_info(file_name='tunnel_info.json'):
    file_name = os.path.join(SSH_FOLDER, file_name)
    if os.path.exists(file_name):
        with open(file_name, 'r') as f:
            tunnel_info = json.load(f)
        return tunnel_info


def handle_ssh_response(ssh_tunnel, password):
    ssh_options = [('Are you sure you want to continue connecting', 'Yes'),
                   ('password', password),
                   (pexpect.EOF, None)]
    response = ssh_tunnel.expect([option[0] for option in ssh_options])
    time.sleep(1)
    if response < 2:
        print("{:d}: Response received: {}  Sending Command: {}".format(response, ssh_options[response][0], ssh_options[response][1]))
        ssh_tunnel.sendline(ssh_options[response][1])  #send the correct option
        handle_ssh_response(ssh_tunnel, password)
    elif response == len(ssh_options) - 1:
        # If the end of stream is found, return succesfully
        pass
    else:
        # If another option was hit raise an error.
        raise Exception("Error: Response was not captured. Run ssh command by hand to investiage issue. Response = {}".format(response))


def create_ssh_tunnel(tunnel_command, password):
    """ Creates a Reverse SSH Tunnel to allow the Cloud to talk to the PFC
    Need to account for the RSA Fingerprint prompt: 
    The authenticity of host '[ec2-34-209-193-14.us-west-2.compute.amazonaws.com]:3004 ([34.209.193.14]:3004)' can't be established.
    RSA key fingerprint is 67:54:e8:e4:d8:ae:e0:61:11:1e:f5:5c:77:d6:83:de.
    Are you sure you want to continue connecting (yes/no)? yes
    """
    #try:
    ssh_tunnel = pexpect.spawn(tunnel_command)
    handle_ssh_response(ssh_tunnel, password)
    time.sleep(10) # Cygwin is slow to update process status.
    #ssh_tunnel.expect (pexpect.EOF)
    print("SSH tunnel created!")
    #except Exception as e:
    #    print(str(e))


def get_pfc_uuid():
    return str(uuid.uuid1())


def get_pfc_name():
    pfc_name = os.uname()[1]
    if not pfc_name:
        pfc_name = "pfc" + str(random.randint(0, 1000))
    return pfc_name


def add_pfc_to_cloud(session, pfc_url):
    """
    Registers the PFC by submitting PFC information to cloud backend.

    Data Required to register pfc.
    {
      "url": "string",
      "name": "string",
      "uuid": "string"
    }
    """
    pfc_data = {}
    pfc_data['url'] = pfc_url   # Provided by cloud backend on ssh tunnel call
    pfc_data['name'] = get_pfc_name()   # Return the PFCs host name
    pfc_data['uuid'] = get_pfc_uuid()   # Return the PFCs UUID based on the Mac Address and Serial Number (hostname)
    API_REGISTRATION_END_POINT = "/pfc"
    res = session.put(SERVER_URL + API_REGISTRATION_END_POINT, json=pfc_data)
    # Add Error checking
    if res.status_code != 201:
        print(res.text)
        raise Exception("Failed to register PFC. Status_code: 201 != {}".format(res.status_code))
def add_pfc(tunnel_info, session=None):
    """
    Adds a PFC to the Users Account
    """
    if not session:
        raise Exception("Requires requests session object to be passed in")
    register_login(session, register_user=False)  # Check if already logged in
    add_pfc_to_cloud(session, tunnel_info['url'])


def main():
    tunnel_info = get_tunnel_info()
    print(tunnel_info)
    save_tunnel_info(tunnel_info)
    # Check for ssh tunnel
    # Create Tunnel if it doesn't exist
    create_ssh_tunnel(tunnel_info['ssh_command'], tunnel_info['password'])
    # Register PFC


def test_add_pfc():
    session = requests.Session()
    tunnel_info = get_tunnel_info()
    add_pfc(tunnel_info, session)


def test_get_pfc_name():
    pfc_name = get_pfc_name()
    assert type(pfc_name) == str
    assert len(pfc_name) < 20


if __name__ == "__main__":
    main()

