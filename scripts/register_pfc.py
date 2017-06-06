import os
import requests
import subprocess
import json
import pdb
import pexpect
from settings import SERVER_URL, SSH_FOLDER, user_account

login_info = {
        "password": user_account['password'],
        "email": user_account['email']
        }


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


def register_login(session):
    REGISTER_URL = "/auth"
    response = session.post(SERVER_URL + REGISTER_URL, json=login_info)
    if 200 <= response.status_code <= 210:
        return True
    else:
        pdb.set_trace()
        print(response.text)
        raise Exception("Failed to Register")


def get_new_tunnel_info(session):
    TUNNEL_URL = '/tunnel'
    response = session.put(SERVER_URL + TUNNEL_URL)
    if response.status_code == 201:
        return response.json()
    else:
        #pdb.set_trace()
        print(response.text)
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


def create_ssh_tunnel(tunnel_command, password):

    try:
        ssh_tunnel = pexpect.spawn(tunnel_command)
        ssh_tunnel.expect('password:')
        time.sleep(1)
        ssh_tunnel.sendline(password)
        time.sleep(10) # Cygwin is slow to update process status.
        ssh_tunnel.expect (pexpect.EOF)
        print("SSH tunnel created!")
    except Exception as e:
        print(str(e))


def main():
    tunnel_info = get_tunnel_info()
    print(tunnel_info)
    save_tunnel_info(tunnel_info)
    create_ssh_tunnel(tunnel_info['ssh_command'], tunnel_info['password'])

if __name__ == "__main__":
    main()

