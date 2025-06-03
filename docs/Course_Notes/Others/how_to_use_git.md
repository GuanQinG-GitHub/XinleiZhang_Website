## Simple workflow with Git
1. Clone the remote repo from Github as origin (default name of local cloned repo)
2. Edit the files in the local repo
3. Add the changes into the Stage
   
    ```sh
    git add . # add all changes into the Stage
    ```

4. Commit the changes in the local repo
   
    ```sh
    git commit -m "message" # commit the changes in the local repo and indicate the changes with the variable message
    ```

5. Push the local repo to the remote repo in Github
   
    ```sh
    git push origin main # push the local repo to the main branch in the remote repo
    ```
6. Combine the current changes into the last commit

    ```sh
    git commit --amend -m "message"
    ```
    Since `--amend` rewrites history, the commit ID will change. If you've already pushed the last commit to a remote repository, you'll need to use `force push` this time

    ```sh
    git push origin main --force
    ```
***Other Commands***
```sh
git remote # show the name of the local cloned repo
git status # show the editing status of Stage, local repo
```

## Join and work in other's project

### 🔐 Setting Up GitHub SSH Access on Ubuntu

#### 1. Generate a New SSH Key

Open your terminal and run:

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

- Replace `"your_email@example.com"` with your GitHub email address.
- Press `Enter` to accept the default file location.
- Optionally, set a passphrase for added security.

### 2. Start the SSH Agent and Add Your Key

Start the SSH agent in the background:

```bash
eval "$(ssh-agent -s)"
```

Add your SSH private key to the agent:

```bash
ssh-add ~/.ssh/id_ed25519
```

### 3. Add the SSH Key to Your GitHub Account

Copy the contents of your public key to the clipboard:

```bash
cat ~/.ssh/id_ed25519.pub
```

Then, follow these steps:

1. Log in to your GitHub account.
2. In the upper-right corner, click your profile photo, then click **Settings**.
3. In the left sidebar, click **SSH and GPG keys**.
4. Click **New SSH key**.
5. In the "Title" field, add a descriptive label for the new key.
6. Paste your public key into the "Key" field.
7. Click **Add SSH key**.

### 4. Test Your SSH Connection

Verify that your SSH key is correctly set up:

```bash
ssh -T git@github.com
```

You should see a message like:

```bash
Hi username! You've successfully authenticated, but GitHub does not provide shell access.
```

### 5. Clone a Repository Using SSH

Now, you can clone repositories using SSH:

```bash
git clone git@github.com:username/repository.git
```
(Just copy from GitHub in the SSH tab)

- Replace `username` with the repository owner's GitHub username.
- Replace `repository` with the name of the repository you want to clone.

---

By following these steps, you've configured SSH authentication with GitHub on your Ubuntu system. This setup allows you to interact with GitHub repositories securely without entering your credentials each time.

