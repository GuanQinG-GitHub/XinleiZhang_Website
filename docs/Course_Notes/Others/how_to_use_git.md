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

